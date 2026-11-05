# Helper code for low-level motion queuing and flushing
#
# Copyright (C) 2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import chelper

BGFLUSH_LOW_TIME = 0.200
BGFLUSH_HIGH_TIME = 0.400
# The embedded K400 host has shown short scheduling gaps that allowed active
# IDEX step batches to reach the MCU after their deadline.  Keep an additional
# 300ms of generated motion queued; pause requests may take up to 300ms longer
# to drain, but the MCU's Timer too close safety check remains unchanged.
BGFLUSH_SG_LOW_TIME = 0.750
BGFLUSH_SG_HIGH_TIME = 1.000
BGFLUSH_EXTRA_TIME = 0.250

# Thresholds for the FlushTiming log line.  Past "Timer too close" shutdowns
# on the embedded K400 host showed a 0.6-0.8s gap in step-command sends while
# load, MCU task time and buffer stats all stayed healthy, so record the
# flush-timer wake lateness, the step-generation batch duration and the
# wall-clock gap between batches to identify which interval stalls.  Normal
# stepping wakes once per 0.250s batch, hence the 0.500s gap threshold.
FLUSH_WAKE_LATE_LOG = 0.100
FLUSH_ADVANCE_LOG = 0.050
FLUSH_SG_GAP_LOG = 0.500

MOVE_HISTORY_EXPIRE = 30.
MIN_KIN_TIME = 0.100
STEPCOMPRESS_FLUSH_TIME = 0.050
SDS_CHECK_TIME = 0.001 # step+dir+step filter in stepcompress.c

DRIP_SEGMENT_TIME = 0.050
DRIP_TIME = 0.100


def _calc_step_gen_target(last_step_gen_time, est_print_time, aggr_sg_time):
    want_sg_time = est_print_time + BGFLUSH_SG_HIGH_TIME
    batch_time = BGFLUSH_SG_HIGH_TIME - BGFLUSH_SG_LOW_TIME
    next_batch_time = last_step_gen_time + batch_time
    # A short host gap can leave one normal batch below the configured low
    # watermark.  Catch up to the high watermark in the same handler so late
    # step commands cannot shut down the MCU; the actual motion horizon still
    # caps the target.  This only restores the configured high watermark, so
    # pause/cancel never exceeds its normal maximum drain window.
    if next_batch_time <= est_print_time + BGFLUSH_SG_LOW_TIME:
        next_batch_time = want_sg_time
    elif next_batch_time > want_sg_time + 0.005:
        # Delay flushing until next wakeup
        next_batch_time = last_step_gen_time
    return min(next_batch_time, aggr_sg_time)


class PrinterMotionQueuing:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        # C trapq tracking
        self.trapqs = []
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        # C steppersync tracking
        self.steppersyncmgr = ffi_main.gc(ffi_lib.steppersyncmgr_alloc(),
                                          ffi_lib.steppersyncmgr_free)
        self.syncemitters = []
        self.syncemitter_to_name = {}
        self.steppersyncs = []
        self.steppersyncmgr_gen_steps = ffi_lib.steppersyncmgr_gen_steps
        # History expiration
        self.clear_history_time = 0.
        # Flush notification callbacks
        self.flush_callbacks = []
        # Kinematic step generation scan window time tracking
        self.kin_flush_delay = SDS_CHECK_TIME
        # MCU tracking
        self.all_mcus = [m for n, m in printer.lookup_objects(module='mcu')]
        self.mcu = self.all_mcus[0]
        self.can_pause = True
        if self.mcu.is_fileoutput():
            self.can_pause = False
        # Flush tracking
        flush_handler = self._flush_handler
        if not self.can_pause:
            flush_handler = self._flush_handler_debug
        self.flush_timer = self.reactor.register_timer(flush_handler)
        self.do_kick_flush_timer = True
        self.last_flush_time = self.last_step_gen_time = 0.
        self.need_flush_time = self.need_step_gen_time = 0.
        # Flush-timer latency tracking (see FLUSH_WAKE_LATE_LOG)
        self.expect_wake_time = 0.
        self.last_advance_monotime = 0.
        # "Drip" timing (for homing and probing moves)
        self.drip_start_times = []
        # Register handlers
        printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
    # C trapq tracking
    def allocate_trapq(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapqs.append(trapq)
        return trapq
    def wipe_trapq(self, trapq):
        # Expire any remaining movement in the trapq (force to history list)
        self.trapq_finalize_moves(trapq, self.reactor.NEVER, 0.)
    def lookup_trapq_append(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        return ffi_lib.trapq_append
    # C steppersync tracking
    def _lookup_steppersync(self, mcu):
        for ss_mcu, ss in self.steppersyncs:
            if ss_mcu is mcu:
                return ss
        ffi_main, ffi_lib = chelper.get_ffi()
        ss = ffi_lib.steppersyncmgr_alloc_steppersync(self.steppersyncmgr)
        self.steppersyncs.append((mcu, ss))
        return ss
    def allocate_syncemitter(self, mcu, name, alloc_stepcompress=True):
        sname = name.split()[-1]
        thread_name = sname.encode("utf-8")[:15]
        ss = self._lookup_steppersync(mcu)
        ffi_main, ffi_lib = chelper.get_ffi()
        se = ffi_lib.steppersync_alloc_syncemitter(ss, thread_name,
                                                   alloc_stepcompress)
        self.syncemitter_to_name[se] = name
        self.syncemitters.append(se)
        return se
    def setup_mcu_movequeue(self, mcu, serialqueue, move_count):
        # Setup steppersync object for the mcu's main movequeue
        ffi_main, ffi_lib = chelper.get_ffi()
        ss = self._lookup_steppersync(mcu)
        ffi_lib.steppersync_setup_movequeue(ss, serialqueue, move_count)
        mcu_freq = float(mcu.seconds_to_clock(1.))
        ffi_lib.steppersync_set_time(ss, 0., mcu_freq)
    def stats(self, eventtime):
        # Globally calibrate mcu clocks (and step generation clocks)
        sync_time = self.last_step_gen_time
        ffi_main, ffi_lib = chelper.get_ffi()
        for mcu, ss in self.steppersyncs:
            offset, freq = mcu.calibrate_clock(sync_time, eventtime)
            ffi_lib.steppersync_set_time(ss, offset, freq)
        # Calculate history expiration
        est_print_time = self.mcu.estimated_print_time(eventtime)
        self.clear_history_time = max(0., est_print_time - MOVE_HISTORY_EXPIRE)
        return False, ""
    # Flush notification callbacks
    def register_flush_callback(self, callback, can_add_trapq=False):
        if can_add_trapq:
            self.flush_callbacks = [callback] + self.flush_callbacks
        else:
            self.flush_callbacks = self.flush_callbacks + [callback]
    def unregister_flush_callback(self, callback):
        if callback in self.flush_callbacks:
            fcbs = list(self.flush_callbacks)
            fcbs.remove(callback)
            self.flush_callbacks = fcbs
    # Kinematic step generation scan window time tracking
    def get_kin_flush_delay(self):
        return self.kin_flush_delay
    def check_step_generation_scan_windows(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        kin_flush_delay = SDS_CHECK_TIME
        for se in self.syncemitters:
            sk = ffi_lib.syncemitter_get_stepper_kinematics(se)
            if sk == ffi_main.NULL:
                continue
            trapq = ffi_lib.itersolve_get_trapq(sk)
            if trapq == ffi_main.NULL:
                continue
            pre_active = ffi_lib.itersolve_get_gen_steps_pre_active(sk)
            post_active = ffi_lib.itersolve_get_gen_steps_post_active(sk)
            kin_flush_delay = max(kin_flush_delay, pre_active, post_active)
        self.kin_flush_delay = kin_flush_delay
    # Flush tracking
    def _handle_shutdown(self):
        self.can_pause = False
    def _advance_flush_time(self, want_flush_time, want_step_gen_time=0.):
        flush_time = max(want_flush_time, self.last_flush_time,
                         want_step_gen_time - STEPCOMPRESS_FLUSH_TIME)
        step_gen_time = max(want_step_gen_time, self.last_step_gen_time,
                            flush_time)
        # Invoke flush callbacks (if any)
        with self.reactor.assert_no_pause():
            for cb in self.flush_callbacks:
                cb(flush_time, step_gen_time)
        # Determine maximum history to keep
        trapq_free_time = step_gen_time - self.kin_flush_delay
        clear_history_time = self.clear_history_time
        if not self.can_pause:
            clear_history_time = max(0., trapq_free_time - MOVE_HISTORY_EXPIRE)
        # Generate stepper movement and transmit
        ret = self.steppersyncmgr_gen_steps(self.steppersyncmgr, flush_time,
                                            step_gen_time, clear_history_time)
        if ret:
            msg = "Internal error in stepcompress"
            queue_name = self.syncemitter_to_name.get(ret)
            err_details = {
                "queue_name": queue_name,
                "flush_time": flush_time, "step_gen_time": step_gen_time,
                "last_flush_time": self.last_flush_time,
                "last_step_gen_time": self.last_step_gen_time,
                "clear_history_time": clear_history_time }
            self.printer.invoke_shutdown(msg, err_details)
            raise self.mcu.error(msg)
        self.last_flush_time = flush_time
        self.last_step_gen_time = step_gen_time
        self.last_advance_monotime = self.reactor.monotonic()
        # Move processed trapq entries to history list, and expire old history
        for trapq in self.trapqs:
            self.trapq_finalize_moves(trapq, trapq_free_time,
                                      clear_history_time)
    def _await_flush_time(self, want_flush_time):
        while 1:
            if self.last_flush_time >= want_flush_time or not self.can_pause:
                return
            systime = self.reactor.monotonic()
            est_print_time = self.mcu.estimated_print_time(systime)
            wait = want_flush_time - BGFLUSH_HIGH_TIME - est_print_time
            if wait <= 0.:
                return
            self.reactor.pause(systime + min(1., wait))
    def flush_all_steps(self):
        flush_time = self.need_step_gen_time
        self._await_flush_time(flush_time)
        self._advance_flush_time(flush_time)
    def calc_step_gen_restart(self, est_print_time):
        kin_time = max(est_print_time + MIN_KIN_TIME, self.last_step_gen_time)
        return kin_time + self.kin_flush_delay
    def _flush_handler(self, eventtime):
        try:
            est_print_time = self.mcu.estimated_print_time(eventtime)
            monotime = self.reactor.monotonic()
            wake_late = 0.
            if self.expect_wake_time:
                wake_late = max(0., eventtime - self.expect_wake_time)
            sg_gap = 0.
            if self.last_advance_monotime:
                sg_gap = monotime - self.last_advance_monotime
            t0 = monotime
            aggr_sg_time = self.need_step_gen_time - 2.*self.kin_flush_delay
            is_stepgen = self.last_step_gen_time < aggr_sg_time
            if is_stepgen:
                # Actively stepping - want more aggressive flushing
                want_sg_time = _calc_step_gen_target(
                    self.last_step_gen_time, est_print_time, aggr_sg_time)
                # Flush motion queues (if needed)
                if want_sg_time > self.last_step_gen_time:
                    self._advance_flush_time(0., want_sg_time)
            else:
                # Not stepping (or only step remnants) - use relaxed flushing
                want_flush_time = est_print_time + BGFLUSH_HIGH_TIME
                max_flush_time = self.need_flush_time + BGFLUSH_EXTRA_TIME
                want_flush_time = min(want_flush_time, max_flush_time)
                # Flush motion queues (if needed)
                if want_flush_time > self.last_flush_time:
                    self._advance_flush_time(want_flush_time)
            advance_time = self.reactor.monotonic() - t0
            if (wake_late > FLUSH_WAKE_LATE_LOG
                    or advance_time > FLUSH_ADVANCE_LOG
                    or sg_gap > FLUSH_SG_GAP_LOG):
                # The first captured incidents showed wake_late=0 and
                # advance<0.002, so report the queue-side limits too: the
                # generation horizon is capped at need_sg - 2*kfd, and only
                # kfd/need_flush can say whether that cap or an empty move
                # queue is what starved step generation.
                logging.warning(
                    "FlushTiming: br=%s wake_late=%.3f sg_gap=%.3f"
                    " advance=%.3f kfd=%.3f est_print_time=%.3f"
                    " last_step_gen=%.3f need_sg=%.3f aggr_sg=%.3f"
                    " last_flush=%.3f need_flush=%.3f",
                    ("sg" if is_stepgen else "idle"), wake_late, sg_gap,
                    advance_time, self.kin_flush_delay, est_print_time,
                    self.last_step_gen_time, self.need_step_gen_time,
                    aggr_sg_time, self.last_flush_time, self.need_flush_time)
            # Reschedule timer
            aggr_sg_time = self.need_step_gen_time - 2.*self.kin_flush_delay
            if self.last_step_gen_time < aggr_sg_time:
                waketime = self.last_step_gen_time - BGFLUSH_SG_LOW_TIME
            else:
                self.do_kick_flush_timer = True
                max_flush_time = self.need_flush_time + BGFLUSH_EXTRA_TIME
                if self.last_flush_time >= max_flush_time:
                    self.expect_wake_time = 0.
                    return self.reactor.NEVER
                waketime = self.last_flush_time - BGFLUSH_LOW_TIME
            delay = eventtime + waketime - est_print_time
            self.expect_wake_time = self.reactor.monotonic() + delay
            return delay
        except:
            logging.exception("Exception in flush_handler")
            self.printer.invoke_shutdown("Exception in flush_handler")
        return self.reactor.NEVER
    def _flush_handler_debug(self, eventtime):
        # Use custom flushing code when in batch output mode
        try:
            faux_time = self.need_flush_time - 1.5
            batch_time = BGFLUSH_SG_HIGH_TIME - BGFLUSH_SG_LOW_TIME
            flush_count = 0
            while self.last_step_gen_time < faux_time:
                target = self.last_step_gen_time + batch_time
                if flush_count > 100. and faux_time > target:
                    target += int((faux_time-target) / batch_time) * batch_time
                self._advance_flush_time(0., target)
                flush_count += 1
            if flush_count:
                return self.reactor.NOW
            self._advance_flush_time(self.need_flush_time + BGFLUSH_EXTRA_TIME)
            self.do_kick_flush_timer = True
            return self.reactor.NEVER
        except:
            logging.exception("Exception in flush_handler_debug")
            self.printer.invoke_shutdown("Exception in flush_handler_debug")
        return self.reactor.NEVER
    def note_mcu_movequeue_activity(self, mq_time, is_step_gen=True):
        if is_step_gen:
            mq_time += self.kin_flush_delay
            self.need_step_gen_time = max(self.need_step_gen_time, mq_time)
        self.need_flush_time = max(self.need_flush_time, mq_time)
        if self.do_kick_flush_timer:
            self.do_kick_flush_timer = False
            self.expect_wake_time = 0.
            # Restart the gap measurement so an idle period is not reported
            # as a step-generation stall on the first wake after the kick.
            self.last_advance_monotime = 0.
            self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
    # "Drip" timing (for homing and probing moves)
    def drip_update_time(self, start_time, end_time, drip_completion):
        self.drip_start_times.append(start_time)
        self._await_flush_time(start_time)
        # Disable background flushing from timer
        self.reactor.update_timer(self.flush_timer, self.reactor.NEVER)
        self.do_kick_flush_timer = False
        self.expect_wake_time = 0.
        self._advance_flush_time(start_time - SDS_CHECK_TIME, start_time)
        # Flush in segments until drip_completion signal
        flush_time = start_time
        while flush_time < end_time:
            if drip_completion.test():
                break
            curtime = self.reactor.monotonic()
            est_print_time = self.mcu.estimated_print_time(curtime)
            wait_time = flush_time - est_print_time - DRIP_TIME
            if wait_time > 0. and self.can_pause:
                # Pause before sending more steps
                drip_completion.wait(curtime + wait_time)
                continue
            flush_time = min(flush_time + DRIP_SEGMENT_TIME, end_time)
            self.note_mcu_movequeue_activity(flush_time)
            self._advance_flush_time(flush_time - SDS_CHECK_TIME, flush_time)
        # Restore background flushing
        self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
        self._advance_flush_time(flush_time + self.kin_flush_delay)
        self.drip_start_times.remove(start_time)
    def check_drip_timing(self):
        if not self.drip_start_times:
            return None
        return min(self.drip_start_times)

def load_config(config):
    return PrinterMotionQueuing(config)
