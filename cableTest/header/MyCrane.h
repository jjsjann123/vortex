#ifndef _MY_CRANE_H
#define _MY_CRANE_H

#include <VxSim/VxExtension.h>

#include <string>

namespace Vx
{
    class VxAssembly;
    class VxHinge;
    class VxPart;
    class VxPrismatic;
}

namespace VxSim
{
    class VxMechanism;
}

class MyCrane
{
public:
    ~MyCrane();
    MyCrane();

    VxSim::VxMechanism* getMechanism() { return mMechanism.get(); }

    void setElevationSpeed(Vx::VxReal iSpeed);
    void setElongationSpeed(Vx::VxReal iSpeed);
    void setWinchSpeed(Vx::VxReal iSpeed);

private:
    VxSim::VxMechanism* createMechanism();

    Vx::VxAssembly* getAssembly();
    Vx::VxAssembly* createAssembly();

    // @internal
    // Functions to create the different parts of the crane.
    Vx::VxPart* createBase();
    Vx::VxPart* createWinch();
    Vx::VxPart* createLowerBoom();
    Vx::VxPart* createUpperBoom();
    Vx::VxPart* createMidPulley();
    Vx::VxPart* createTipPulley();

    void createConstraints();


    VxSim::VxExtension* createKeyboardExtension();

public:
    static const std::string sCraneAssemblyName;
    static const std::string sWinchName;
    static const std::string sMidPulleyName;
    static const std::string sTipPulleyName;

private:
    // References to the concrete (objects) in order to modify their behavior during onPreUpdate()
    Vx::VxSmartPtr<VxSim::VxMechanism> mMechanism;


    // The constraint to link the parts together
    // It also is used to move the boom.
    Vx::VxHinge*     mHingeForElevation;
    Vx::VxPrismatic* mPrismaticForElongation;

    // It will also be used to winch the cable in and out.
    Vx::VxHinge* mHingeForWinch;

    // The crane moves by pressing keyboard keys.
    Vx::VxSmartPtr<VxSim::VxExtension> mKeyboard;
};

#endif
