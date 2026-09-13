//! Submission ordering for reusable ISO transfer slots.

extern crate alloc;

use alloc::collections::VecDeque;
use core::task::Poll;

#[derive(Default)]
pub(super) struct CompletionOrder {
    pending: VecDeque<usize>,
}

impl CompletionOrder {
    pub(super) fn submitted(&mut self, slot: usize) {
        self.pending.push_back(slot);
    }

    pub(super) fn poll<T>(
        &mut self,
        mut poll_slot: impl FnMut(usize) -> Poll<T>,
    ) -> Poll<(usize, T)> {
        let Some(&slot) = self.pending.front() else {
            return Poll::Pending;
        };
        // Recycled slot indices carry no chronology. Only the oldest submitted
        // request can supply the next payload, even if later requests are ready.
        poll_slot(slot).map(|completion| {
            self.pending.pop_front();
            (slot, completion)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recycled_slots_cannot_overtake_older_payloads() {
        let mut order = CompletionOrder::default();
        for slot in 0..4 {
            order.submitted(slot);
        }
        // All requests complete before the task runs. Requeued slots complete
        // immediately too: slot index must not become a scheduling priority.
        for sequence in 0..12 {
            let expected_slot = sequence % 4;
            assert_eq!(
                order.poll(|slot| Poll::Ready(slot)),
                Poll::Ready((expected_slot, expected_slot))
            );
            order.submitted(expected_slot);
        }
    }

    #[test]
    fn later_completion_waits_for_the_oldest_request() {
        let mut order = CompletionOrder::default();
        order.submitted(0);
        order.submitted(1);
        assert_eq!(
            order.poll(|slot| if slot == 0 {
                Poll::Pending
            } else {
                Poll::Ready(())
            }),
            Poll::Pending
        );
        assert_eq!(order.poll(|_| Poll::Ready(())), Poll::Ready((0, ())));
        assert_eq!(order.poll(|_| Poll::Ready(())), Poll::Ready((1, ())));
        assert_eq!(order.poll(|_| Poll::Ready(())), Poll::Pending);
    }
}
