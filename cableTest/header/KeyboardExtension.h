#ifndef _KEYBOARD_EXTENSION_H
#define _KEYBOARD_EXTENSION_H

#include "MyCrane.h"
#include <VxSim/IKeyboard.h>
#include <VxSim/IExtension.h>
#include <Vx/VxParameter.h>

//  Keyboard extension class
class KeyboardExtension : public VxSim::IKeyboard, public VxSim::IExtension
{
public:
    // Destructor 
    virtual ~KeyboardExtension();

    // Constructor
    KeyboardExtension(VxSim::VxPluginExtension *iProxy);

    // Callback when there is a key pressed. The key argument is
    // a key-code that combines an eKeyCode and possibly an eKeyModifier.
    //
    virtual void onKeyPressed(int key);

    // Called when there is a key released. The key argument is
    // a key-code that combines an eKeyCode and possibly an eKeyModifier.
    // 
    virtual void onKeyReleased(int key);

    // Set the crane to be controlled by this Keyboard extension
    //
    void setCrane(MyCrane * iCrane);

private:

    // This is the crane to be controlled by this extension.
    MyCrane * mCrane;

    Vx::VxReal mInc;
};

#endif // _KEYBOARD_HOOK_EXTENSION_H
