#include "utils/Observer.hpp"

using namespace uts;

void Subject::attach(IObserver* observer)
{
    if (observer == nullptr) return;
    if (std::find(observers_.begin(), observers_.end(), observer) == observers_.end()) {
        observers_.push_back(observer);
    }
}

void Subject::detach(IObserver* observer)
{
    observers_.erase(std::remove(observers_.begin(), observers_.end(), observer),
                    observers_.end());
}

void Subject::notifyAll(const TrafficNotification& notification) 
{
    // Copy: an observer may detach itself while being notified.
    std::vector<IObserver*> snapshot = observers_;
    for (IObserver* observer : snapshot) {
        observer->onNotify(notification);
    }
}