#include "KeyboardExtension.h"

#include <Vx/VxHinge.h>
#include <Vx/VxPart.h>

// Default Destructor
KeyboardExtension::~KeyboardExtension()
{
}

// Default Constructor
KeyboardExtension::KeyboardExtension(VxSim::VxPluginExtension *iProxy)
    : VxSim::IKeyboard(iProxy)
    , VxSim::IExtension(iProxy)
    , mCrane(NULL)
    , mInc(0.2)
{
    addKeyDescription(IKeyboard::kShiftMask + '7', "Hold to extend the telescopic section of the crane (i.e. Boom out)");
    addKeyDescription('7', "Hold to retract the telescopic section of the crane (i.e. Boom in)");
    addKeyDescription(IKeyboard::kShiftMask + '8', "Hold to lift the boom of the crane (i.e. Boom up)");
    addKeyDescription('8', "Hold tso lower the boom of the crane (i.e. Boom down)");
    addKeyDescription(IKeyboard::kShiftMask + '9', "Hold to winch out the cable of the crane (i.e. Winch out)");
    addKeyDescription('9', "Hold to winch in the cable of the crane (i.e. Winch in)");
    addKeyDescription(IKeyboard::kAltMask, "Holding the alt key while using the previous key will multiply the speed by 2");
}

// Increase or decrease the speed of the different values for the control of
// the crane.
void KeyboardExtension::onKeyPressed(int key)
{
    const Vx::VxReal factor = (key & IKeyboard::kShiftMask ? 1.0 : -1.0) *  (key & IKeyboard::kAltMask ? 2.0 : 1.0);



    if(mCrane)
    {
        switch(key & ~(IKeyboard::kShiftMask | IKeyboard::kAltMask) )
        {
        case '7': // Boom In or Out (extend)
            mCrane->setElongationSpeed(factor * 0.5);
            break;

        case '8' : // Boom Up or Down
            mCrane->setElevationSpeed(factor * 0.1);
            break;

        case '9' : // Winch In or Out
            mCrane->setWinchSpeed(factor * 0.5);
            break;

		case 'a' :
			std::cout << "test if it works" << std::endl;
			break;
        }
    }
}

void KeyboardExtension::onKeyReleased(int key)
{
    if(mCrane)
    {
        switch( key  & ~(IKeyboard::kShiftMask | IKeyboard::kAltMask) )
        {
        case '7' : // Boom In or Out (extend)
            mCrane->setElongationSpeed(0.0);
            break;

        case '8': // Boom Up or Down
            mCrane->setElevationSpeed(0.0);
            break;

        case '9': // Winch In or Out
            mCrane->setWinchSpeed(0.0);
            break;
        }
    }
}

void KeyboardExtension::setCrane(MyCrane * iCrane)
{
    mCrane = iCrane;
}
