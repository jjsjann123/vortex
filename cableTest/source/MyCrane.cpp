#include "MyCrane.h"
#include "KeyboardExtension.h"

#include <VxSim/VxExtensionFactory.h>
#include <VxSim/VxFactoryKey.h>
#include <VxSim/VxMechanism.h>
#include <VxSim/VxUuid.h>

#include <Vx/Find.h>
#include <Vx/VxAssembly.h>
#include <Vx/VxBox.h>
#include <Vx/VxCollisionGeometry.h>
#include <Vx/VxCollisionRule.h>
#include <Vx/VxCylinder.h>
#include <Vx/VxHinge.h>
#include <Vx/VxPart.h>
#include <Vx/VxPrismatic.h>

using namespace Vx;
using namespace VxSim;

const std::string MyCrane::sCraneAssemblyName("CraneAssembly");
const std::string MyCrane::sWinchName("Winch");
const std::string MyCrane::sMidPulleyName("MidPulley");
const std::string MyCrane::sTipPulleyName("TipPulley");


// Default destructor
// Delete the mechanism and the keyboard.
MyCrane::~MyCrane()
{
}


// Default constructor
// Create the crane with everything it needs.
// The crane has a base defined as static since it should not move.
// The boom is composed of 2 parts; each part can move with respect to
// each other. This make the boom extendable.
// The boom is attached to the base by an Hinge and it can rotate.
// The upper part of the boom is attached to the lower part by a Prismatic,
// hence the boom can extend in the direction of the boom
// To help move the cable on the boom, two pulleys are attached to the boom:
// one in the middle and one at the tip.
//
// The crane also has a winch on the lower part of the boom from where
// the cable spools out.
//
// The user is able to control the crane by pressing key on the keyboard.
MyCrane::MyCrane()
    : mKeyboard(NULL)
{

    mMechanism = createMechanism();
    // Create the constraints to enable motion between the different part of the crane.
    createConstraints();

    // Create the keyboard extension since the crane will be moved with the
    // keyboard keys.
    mKeyboard = createKeyboardExtension();
    mMechanism->add(mKeyboard.get());
}


// Create the crane mechanism to be added to the scene.
VxSim::VxMechanism* MyCrane::createMechanism()
{
    // Create the mechanism to be able to add the assembly.
    VxSim::VxMechanism* mechanism = new VxSim::VxMechanism();
    mechanism->setName("CraneMechanism");

    // Create the assembly and add it to the mechanism.
    mechanism->addAssembly(createAssembly());

    return mechanism;
}


// Create the assembly for the crane and add the different parts.
// The parts are Vortex VxPart objects and they are added to the Vortex universe.
VxAssembly* MyCrane::createAssembly()
{
    VxAssembly* assembly = new VxAssembly();
    assembly->setName(sCraneAssemblyName.c_str());

    VxPart* base = createBase();
    assembly->addPart(base);

    VxPart* winch = createWinch();
    assembly->addPart(winch);

    VxPart* lowerBoom = createLowerBoom();
    assembly->addPart(lowerBoom);

    VxPart* upperBoom = createUpperBoom();
    assembly->addPart(upperBoom);

    VxPart* midPulley = createMidPulley();
    assembly->addPart(midPulley);

    VxPart* tipPulley = createTipPulley();
    assembly->addPart(tipPulley);

    // The collision between the parts of the assembly is disabled
    // since some of the parts overlap in order to have a nice mechanism.
    // This does not affect the simulation since there will be limits on
    // the different joints that will make "real" collision impossible.
    assembly->appendCollisionRule(Vx::VxCollisionRule(assembly, assembly, false));

    return assembly;
}


// Create the crane's winch and collision geometry to
// be able to see the winch since a VxPart has no real physical substance.
VxPart* MyCrane::createWinch()
{
    // Create the winch of the boom
    VxPart* winch = new VxPart();
    winch->setName(sWinchName.c_str());
    winch->setControl(Vx::VxPart::kControlDynamic);
    winch->setPosition(0.0, 0.0, 8.0);

    VxTransform tm = VxTransform::createRotationFromEulerAngles(0.0, VX_HALF_PI, 0.0);

    const VxReal r=1.9, h=1.0;
    VxCollisionGeometry* drum = new VxCollisionGeometry(new VxCylinder(r, h));
    // The long axis of the cylinder is along its local z axis.
    // Set it parallel to the x axis of the base.
    drum->setTransformRelative( tm );
    winch->addCollisionGeometry(drum);


    VxCollisionGeometry* leftLipWinch = new VxCollisionGeometry(new VxCylinder(1.15*r, 0.35));
    tm.t() = VxVector3(0.325, 0.0, 0.0);
    leftLipWinch->setTransformRelative( tm );
    winch->addCollisionGeometry(leftLipWinch);

    VxCollisionGeometry* rightLipWinch = new VxCollisionGeometry(new VxCylinder(1.15*r, 0.35));
    tm.t() = VxVector3(-0.325, 0.0, 0.0);
    rightLipWinch->setTransformRelative( tm );
    winch->addCollisionGeometry(rightLipWinch);

    return winch;
}


// Create the lower boom of the crane and create the collision geometry to
// be able to see it since a VxPart has no real physical substance.
// The boom will be attached to the base later. It will support the
// winch and the upper part of the boom
VxPart* MyCrane::createLowerBoom()
{
    // Create the lower boom such that it is
    // parallel to the ground. It will be moved later.
    VxPart* lowerBoom = new VxPart();
    lowerBoom->setControl(Vx::VxPart::kControlDynamic);
    lowerBoom->setName("lowerBoom");
    lowerBoom->setPosition(0.0, 0.0, 8.0);

    // The lower boom has 3 collision geometries to look nice and make
    // room for the winch.
    VxCollisionGeometry* rightSideLowerBoom = new VxCollisionGeometry(new VxBox(0.4, 3.0, 2.0));
    rightSideLowerBoom->setTransformRelative(VxTransform::createTranslation(VxVector3(0.75, 0.5, 0.0)));
    lowerBoom->addCollisionGeometry(rightSideLowerBoom);

    VxCollisionGeometry* leftSideLowerBoom = new VxCollisionGeometry(new VxBox(0.4, 3.0, 2.0));
    leftSideLowerBoom->setTransformRelative(VxTransform::createTranslation(VxVector3(-0.75, 0.5, 0.0)));
    lowerBoom->addCollisionGeometry(leftSideLowerBoom);

    VxCollisionGeometry* endLowerBoom = new VxCollisionGeometry(new VxBox(2.0, 7.0, 2.0));
    endLowerBoom->setTransformRelative(VxTransform::createTranslation(VxVector3(0.0, 5.5, 0.0)));
    lowerBoom->addCollisionGeometry(endLowerBoom);

    return lowerBoom;
}


// Create the upper boom of the crane and the collision geometry to
// be able to see it since a VxPart has no real physical substance.
// The upper boom will be attached to the lower boom later. It will support
// mid pulley and the pulley at the tip.
VxPart* MyCrane::createUpperBoom()
{
    // Create the upper boom
    VxPart* upperBoom = new VxPart();
    upperBoom->setControl(Vx::VxPart::kControlDynamic);
    upperBoom->setName("upperBoom");
    // The local frame of the upper boom is located at the end of the lower boom.
    // The setup of the prismatic joint will be simpler.
    upperBoom->setPosition(0.0, 9.0, 8.0);

    // The lower section of the upper boom has 3 collision geometries to look nice and make
    // room for the mid pulley.
    VxCollisionGeometry* startUpperBoom = new VxCollisionGeometry(new VxBox(1.8, 10, 1.8));
    startUpperBoom->setTransformRelative(VxTransform::createTranslation(VxVector3(0.0, -1.0, 0.0)));
    upperBoom->addCollisionGeometry(startUpperBoom);

    VxCollisionGeometry* rightSideUpperBoom = new VxCollisionGeometry(new VxBox(0.4, 2.0, 1.8));
    rightSideUpperBoom->setTransformRelative(VxTransform::createTranslation(VxVector3(0.7, 5.0, 0.0)));
    upperBoom->addCollisionGeometry(rightSideUpperBoom);

    VxCollisionGeometry* leftSideUpperBoom = new VxCollisionGeometry(new VxBox(0.4, 2.0, 1.8));
    leftSideUpperBoom->setTransformRelative(VxTransform::createTranslation(VxVector3(-0.7, 5.0, 0.0)));
    upperBoom->addCollisionGeometry(leftSideUpperBoom);


    // The upper section of the upper boom has 5 collision geometries to look nice and make
    // room for the mid and tip pulleys.
    const VxReal angle = DegreeToRadian(10.0);

    VxTransform tm(VxVector3(0,0,0), VxEulerAngles(-angle, 0.0, 0.0, VxEulerAngles::kXYZ_CounterClockwise_Rotating));

    VxCollisionGeometry* rightSideStartMidUpperBoom = new VxCollisionGeometry(new VxBox(0.4, 2.0, 1.8));
    tm.t() = VxVector3(0.7, 6.0 + 1.0 * cos(angle), -1.0 * sin(angle));
    rightSideStartMidUpperBoom->setTransformRelative( tm );
    upperBoom->addCollisionGeometry(rightSideStartMidUpperBoom);

    VxCollisionGeometry* leftSideStartMidUpperBoom = new VxCollisionGeometry(new VxBox(0.4, 2.0, 1.8));
    tm.t() = VxVector3(-0.7, 6.0 + 1.0 * cos(angle), -1.0 * sin(angle));
    leftSideStartMidUpperBoom->setTransformRelative( tm );
    upperBoom->addCollisionGeometry(leftSideStartMidUpperBoom);


    VxCollisionGeometry* midUpperBoom = new VxCollisionGeometry(new VxBox(1.8, 6, 1.8));
    tm.t() = VxVector3(0.0, 6 + 5.0 * cos(angle), - 5.0 * sin(angle));
    midUpperBoom->setTransformRelative( tm );
    upperBoom->addCollisionGeometry(midUpperBoom);


    VxCollisionGeometry* rightSideEndMidUpperBoom = new VxCollisionGeometry(new VxBox(0.4, 2.0, 1.8));
    tm.t() = VxVector3(0.7, 6.0 + 9.0 * cos(angle), -9.0 * sin(angle));
    rightSideEndMidUpperBoom->setTransformRelative( tm );
    upperBoom->addCollisionGeometry(rightSideEndMidUpperBoom);

    VxCollisionGeometry* leftSideEndMidUpperBoom = new VxCollisionGeometry(new VxBox(0.4, 2.0, 1.8));
    tm.t() = VxVector3(-0.7, 6.0 + 9.0 * cos(angle), -9.0 * sin(angle));
    leftSideEndMidUpperBoom->setTransformRelative( tm );
    upperBoom->addCollisionGeometry(leftSideEndMidUpperBoom);


    return upperBoom;
}


VxPart* MyCrane::createMidPulley()
{
    // Create the pulley at the mid section of the boom.
    VxPart* pulley = new VxPart();
    pulley->setControl(Vx::VxPart::kControlDynamic);
    pulley->setName(sMidPulleyName.c_str());
    pulley->setPosition(0, 15, 8.0);


    // The pulley is composed from 3 Collision geometries to make it look nice.
    const VxReal r=1.0, h=0.8;
    VxCollisionGeometry* drum = new VxCollisionGeometry(new VxCylinder(r, h));
    // The long axis of the cylinder is along its local z axis.
    // Set it parallel to the x axis of the base.
    VxTransform tm = VxTransform::createRotationFromEulerAngles(0.0, VX_HALF_PI, 0.0);

    drum->setTransformRelative(tm);
    pulley->addCollisionGeometry(drum);

    VxCollisionGeometry* leftLipDrum = new VxCollisionGeometry(new VxCylinder(1.15*r, 0.25));
    // The long axis of the cylinder is along its local z axis.
    // Set it parallel to the x axis of the base.
    tm.t() = VxVector3(0.275, 0.0, 0.0);
    leftLipDrum->setTransformRelative( tm );
    pulley->addCollisionGeometry(leftLipDrum);

    VxCollisionGeometry* rightLipDrum = new VxCollisionGeometry(new VxCylinder(1.15*r, 0.25));
    // The long axis of the cylinder is along its local z axis.
    // Set it parallel to the x axis of the base.
    tm.t() = VxVector3(-0.275, 0.0, 0.0);
    rightLipDrum->setTransformRelative( tm );
    pulley->addCollisionGeometry(rightLipDrum);

    return pulley;
}


VxPart* MyCrane::createTipPulley()
{
    const VxReal angle = DegreeToRadian(10.0);

    // Create the pulley at the tip of the boom.
    VxPart* pulley = new VxPart();
    pulley->setControl(Vx::VxPart::kControlDynamic);
    pulley->setName(sTipPulleyName.c_str());
    pulley->setPosition(0, 15.0 + 10.0 * cos(angle) , 8.0 - 10.0 * sin(angle));

    VxTransform tm = VxTransform::createRotationFromEulerAngles(0.0, VX_HALF_PI, 0.0);

    // The pulley is composed from 3 Collision geometries to make it look nice.
    const VxReal r=1.0, h=0.8;
    VxCollisionGeometry* drum = new VxCollisionGeometry(new VxCylinder(r, h));
    // The long axis of the cylinder is along its local z axis.
    // Set it parallel to the x axis of the base.
    drum->setTransformRelative( tm );
    pulley->addCollisionGeometry(drum);

    VxCollisionGeometry* leftLipDrum = new VxCollisionGeometry(new VxCylinder(1.2*r, 0.25));
    // The long axis of the cylinder is along its local z axis.
    // Set it parallel to the x axis of the base.
    tm.t() = VxVector3(0.275, 0.0, 0.0);
    leftLipDrum->setTransformRelative( tm );
    pulley->addCollisionGeometry(leftLipDrum);

    VxCollisionGeometry* rightLipDrum = new VxCollisionGeometry(new VxCylinder(1.2*r, 0.25));
    // The long axis of the cylinder is along its local z axis.
    // Set it parallel to the x axis of the base.
    tm.t() = VxVector3(-0.275, 0.0, 0.0);
    rightLipDrum->setTransformRelative( tm );
    pulley->addCollisionGeometry(rightLipDrum);

    return pulley;
}


void MyCrane::createConstraints()
{
    // All the rotation axes of the different pulleys are in the same direction.
    // This will be reused.
    const VxVector3 axis(1.0, 0.0, 0.0);

    VxAssembly* assembly = getAssembly(); // The pointer returned is always valid.


    VxPart* base = Vx::Find::part("base", assembly);
    VX_ASSERT(NULL != base, "A part named \"base\" must be in the crane assembly.\n");
    VxPart* winch = Vx::Find::part(sWinchName, assembly);
    VX_ASSERT(NULL != winch, "A part named \"winch\" must be in the crane assembly.\n");

    // This constraint is motorized to enable the spooling of the cable.
    mHingeForWinch = new VxHinge(base, winch, winch->getPosition(), axis);
    mHingeForWinch->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlMotorized);
    mHingeForWinch->setMotorDesiredVelocity(VxHinge::kAngularCoordinate, 0.0);
    assembly->addConstraint(mHingeForWinch);


    // Create the two hinges for the pulleys on the upper boom.
    VxPart* upperBoom = Vx::Find::part("upperBoom", assembly);
    VX_ASSERT(NULL != upperBoom, "A part named \"upperBoom\" must be in the crane assembly.\n");


    VxPart* midPulley = Vx::Find::part(sMidPulleyName, assembly);
    VX_ASSERT(NULL != midPulley, "A part named \"midPulley\" must be in the crane assembly.\n");
    // This is not motorized since it is only use to guide the cable.
    VxConstraint* hingeForMidPulley = new VxHinge(upperBoom, midPulley, midPulley->getPosition(), axis);
    hingeForMidPulley->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlFree);
    assembly->addConstraint(hingeForMidPulley);


    VxPart* tipPulley = Vx::Find::part(sTipPulleyName, assembly);
    VX_ASSERT(NULL != base, "A part named \"tipPulley\" must be in the crane assembly.\n");
    // This is not motorized since it is only use to guide the cable.
    VxConstraint* hingeForTipPulley = new VxHinge(upperBoom, tipPulley, tipPulley->getPosition(), axis);
    hingeForTipPulley->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlFree);
    assembly->addConstraint(hingeForTipPulley);

    // Make the constraint to move the crane.

    // Move the boom up or down.
    VxPart* lowerBoom = Vx::Find::part("lowerBoom", assembly);
    VX_ASSERT(NULL != lowerBoom, "A part named \"lowerBoom\" must be in the crane assembly.\n");
    // -axis to have a positive value for the motor when booming up.
    mHingeForElevation = new VxHinge(base, lowerBoom, winch->getPosition(), -axis);
    mHingeForElevation->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlMotorized);
    mHingeForElevation->setMotorDesiredVelocity(VxHinge::kAngularCoordinate, 0.0);
    mHingeForElevation->setLowerLimit(VxHinge::kAngularCoordinate, 0.0);
    mHingeForElevation->setUpperLimit(VxHinge::kAngularCoordinate, VX_HALF_PI - DegreeToRadian(5.0));
    mHingeForElevation->setLimitsActive(VxHinge::kAngularCoordinate, true);
    assembly->addConstraint(mHingeForElevation);

    mPrismaticForElongation = new VxPrismatic(upperBoom, lowerBoom, upperBoom->getPosition(), VxVector3(0.0, 1.0, 0.0));
    mPrismaticForElongation->setControl(VxPrismatic::kLinearCoordinate, VxConstraint::kControlMotorized);
    mPrismaticForElongation->setMotorDesiredVelocity(VxPrismatic::kLinearCoordinate, 0.0);
    mPrismaticForElongation->setLowerLimit(VxPrismatic::kLinearCoordinate, -4.0);
    mPrismaticForElongation->setUpperLimit(Vx::VxPrismatic::kLinearCoordinate, 4.0);
    mPrismaticForElongation->setLimitsActive(VxPrismatic::kLinearCoordinate, true);
    assembly->addConstraint(mPrismaticForElongation);
}


// Create the base of the boom
//
// Returns the new base. The caller owns the pointer.
VxPart* MyCrane::createBase()
{
    VxPart* base = new VxPart();
    // The base is static since it should not move.
    base->setControl(Vx::VxPart::kControlStatic);
    base->setName("base");
    base->setPosition(0.0, 0.0, 6.0);

    VxCollisionGeometry* leftSideBase = new VxCollisionGeometry(new VxBox(1.0, 4.0, 8.0));
    leftSideBase->setTransformRelative(VxTransform::createTranslation(1.5, 0.0, -2.0));
    base->addCollisionGeometry(leftSideBase);

    VxCollisionGeometry* rightSideBase = new VxCollisionGeometry(new VxBox(1.0, 4.0, 8.0));
    rightSideBase->setTransformRelative(VxTransform::createTranslation(-1.5, 0.0, -2.0));
    base->addCollisionGeometry(rightSideBase);

    VxCollisionGeometry* bottom = new VxCollisionGeometry(new VxBox(2.0, 4.0, 1.0));
    bottom->setTransformRelative(VxTransform::createTranslation(0.0, 0.0, -5.5));
    base->addCollisionGeometry(bottom);

    return base;
}


// Get a valid assembly.
// It should always return a valid assembly.
VxAssembly* MyCrane::getAssembly()
{
    const size_t count = mMechanism->getAssemblyCount();
    for (size_t i=0; i<count; ++i)
    {
        VxAssembly* assembly = mMechanism->getAssembly(i);
        if ( sCraneAssemblyName == assembly->getName() )
        {
            return assembly;
        }
    }

    VxWarning(0, "The first assembly of the mechanism, should be the crane assembly. The mechanism might not behave properly.\n");
    VxAssembly* assembly = createAssembly();
    VX_ASSERT(NULL != assembly , "Not able to create a new assembly.\n");

    return assembly;
}

// Update the different speeds of the constraints before a step.
void MyCrane::setElevationSpeed(VxReal iSpeed)
{
    mHingeForElevation->setMotorDesiredVelocity(VxHinge::kAngularCoordinate, iSpeed);
}

void MyCrane::setElongationSpeed(VxReal iSpeed)
{
    mPrismaticForElongation->setMotorDesiredVelocity(VxPrismatic::kLinearCoordinate, iSpeed);
}

void MyCrane::setWinchSpeed(VxReal iSpeed)
{
    mHingeForWinch->setMotorDesiredVelocity(VxHinge::kAngularCoordinate, iSpeed);
}

// Create the keyboard extension to enable the control of the crane by
// pressing keys.
VxSim::VxExtension* MyCrane::createKeyboardExtension()
{
    // Register the KeyboardExtension 
    VxSim::VxFactoryKey key(VxSim::VxUuid("5cb789d0-5585-5d40-8db9-20f499692507"), "Tutorials", "KeyboardExtension");
    VxSim::VxExtensionFactory::registerType<KeyboardExtension>(key);

    // Create the KeyboardExtension that was registered and initialize it.
    VxSim::VxExtension* keyboard = VxExtensionFactory::create(key);
    KeyboardExtension * myKB = dynamic_cast<KeyboardExtension*>(dynamic_cast<VxSim::VxPluginExtension*>(keyboard)->getIExtension());
    if(myKB)
    {
        myKB->setCrane(this);
    }
    return keyboard;
}

