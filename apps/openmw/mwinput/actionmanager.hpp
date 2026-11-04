#ifndef MWINPUT_ACTIONMANAGER_H
#define MWINPUT_ACTIONMANAGER_H

#include <functional>

namespace MWInput
{
    class BindingsManager;

    class ActionManager
    {
    public:
        ActionManager(BindingsManager* bindingsManager, std::function<void()> takeScreenshot);

        void update(float dt);

        void executeAction(int action);

        bool checkAllowedToUseItems() const;

        void toggleMainMenu();
        void toggleConsole();
        void screenshot();
        void activate();
        void rest();
        void quickLoad();
        void quickSave();

        void quickKey(int index);

        void resetIdleTime();
        float getIdleTime() const { return mTimeIdle; }

        bool isSneaking() const;

    private:
        void handleGuiArrowKey(int action);
        void updateCarrying(float dt);

        BindingsManager* mBindingsManager;
        std::function<void()> mTakeScreenshot;

        float mTimeIdle;

        // Activate pressed on a carryable object: a tap activates it, holding the key picks it up to carry.
        // Carrying is a toggle: the next activate press drops the item, attack throws it.
        bool mActivatePending = false;
        float mActivateHeldTime = 0;
        bool mCarrying = false;
        bool mUseWasHeld = false;
        // After throwing, the attack button stays suppressed until it is released.
        bool mSuppressAttackUntilReleased = false;
    };
}
#endif
