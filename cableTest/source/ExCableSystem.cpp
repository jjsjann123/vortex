#include "ExCableSystem.h"
#include "MyCrane.h"

#include <CableSystems/CableSystemsICD.h>
#include <CableSystems/DynamicsICD.h>

#ifdef USE_OSG
#include <CableSystems/GraphicsICD.h>

#include <VxGraphics/ICamera.h>
#include <VxGraphics/IGraphicICD.h>
#include <VxGraphicsPlugins/LightICD.h>
#include <VxGraphicsPlugins/PerspectiveICD.h>
#endif

#include <VxPluginSystem/VxPluginSystem.h>

#include <VxSim/VxApplication.h>
#include <VxSim/VxExtensionFactory.h>

#include <VxData/Container.h>
#include <VxData/FieldArray.h>
#include <VxData/FieldBase.h>

#include <Vx/Find.h>
#include <Vx/VxAssembly.h>
#include <Vx/VxBox.h>
#include <Vx/VxCollisionGeometry.h>
#include <Vx/VxConnectionFactory.h>
#include <Vx/VxHinge.h>
#include <Vx/VxMessage.h>
#include <Vx/VxPart.h>
#include <Vx/VxTransform.h>

#include <string>

using namespace Vx;
using namespace CableSystems;
using namespace CableSystems::DynamicsICD;

static const std::string sLoadAssemblyName("LoadAssembly");
static const std::string sLoadName("Load");
static const std::string sMyDynamicsExtensionName("My CableSystems Dynamics extension");
static const std::string sMyGraphicsExtensionName("My CableSystems Graphics extension");

// The main focus of this tutorial is to show how to use the CableSystems ICD to create a
// cable system for a simple crane.
//
// This tutorial goes through the steps to create mechanisms and add them to a scene.
// Three mechanisms are created:
// 1- A simple crane, with an extendable boom and a hinge to move the boom up and down.
//    The crane also has a controllable winch to spool cable in and out.
// 2- A simple load, which is a part with a box as a collision geometry.

// 3- Finally, a ground is created to support the crane and the load.
//
// After the creation of the mechanisms, a cable system is created.
// The cable system starts at the crane's winch and ends at the load.
// It passes over the two pulleys of the crane before ending at the load.
//


// Go through all the assemblies in the mechanism and return
// the first one with the given name as parameter.
//
// @param[IN] mech  The mechanism in which the assembly should be found
// @param[IN] name  The name of the assembly to find
//
// Returns the found assembly or NULL.
static VxAssembly* GetAssemblyInMechanismFromName(const VxSim::VxMechanism* mech, const std::string& name)
{
    const size_t count = mech->getAssemblyCount();
    for (size_t i=0; i<count; ++i)
    {
        VxAssembly* assembly = mech->getAssembly(i);
        if ( name == assembly->getName() )
        {
            return assembly;
        }
    }

    return NULL;
}

// The object is created only to setup the tutorial.
ExCableSystem::ExCableSystem()
    : mScene()
    , mCrane(NULL)
    , mLoadMechanism()
{
    // Create the scene with the different mechanisms;
    // i.e., crane, load, ground.
    mScene = _createScene();

    // The crane mechanism is created. It is now possible to create the cable system
    // with respect to the parts inside the crane.
   _createCableSystemForCrane();

#ifdef USE_OSG
   // The cable system is displayed with a graphic extension since it is not
   // in Vortex by default, unlike the collision geometries.
   VxSim::VxExtension* gfxExtension = VxSim::VxExtensionFactory::create(CableSystems::GraphicsICD::kFactoryKey);
   VxAssert(NULL != gfxExtension, "Cannot create the CableSystem Graphic Plugin\n");
   gfxExtension->setName(sMyGraphicsExtensionName.c_str());

   mCrane->getMechanism()->add(gfxExtension);

   VxSim::VxExtension* dynExtension = mCrane->getMechanism()->findExtension(sMyDynamicsExtensionName);

   VxConnectionFactory::create(dynExtension->getOutput(kCablesID), gfxExtension->getInput(kCablesID));
#endif
}

// Default destructor
ExCableSystem::~ExCableSystem()
{
    delete mCrane;
}

// Return the created scene
VxSim::VxScene* ExCableSystem::getScene()
{
    return mScene.get();
}

// Create the scene with the different mechanisms;
// i.e., crane, load, ground.
VxSim::VxScene* ExCableSystem::_createScene()
{
    VxSim::VxScene* scene = new VxSim::VxScene();

    // The crane object contains the mechanism for the crane; it also enables the
    // motion of the crane.
    mCrane = new MyCrane();
    scene->add(mCrane->getMechanism());

#ifdef USE_OSG
    // Create a light and add it to the scene
    Vx::VxSmartPtr<VxSim::VxExtension> light = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::LightICD::kDirectionalLightFactoryKey);
    light->getInput(VxGraphicsPlugins::IGraphicICD::kInputOrientation)->setValue(Vx::VxVector3(VX_DEG2RAD(150.0), 0.0, 0.0));
    scene->add(light.get());
#endif

    // Create the load in its own mechanism.
    mLoadMechanism = _createLoad();
    scene->add(mLoadMechanism.get());

    // Create a mechanism for the ground.
    scene->add(_createGround());

/*************
 * J's test
 */
	VxSim::VxMechanism* jMechanism = new VxSim::VxMechanism();
	
	VxAssembly* jAssembly = new VxAssembly();
	jAssembly->setName("jTest");
	VxPart* jPart = new VxPart(200.0);
	_jTestPart = jPart;
	jAssembly->addPart(jPart);
	//jPart->setControl(Vx::VxPart::kControlStatic);
	//jPart->setControl(Vx::VxPart::kControlDynamic);
	jPart->setControl(Vx::VxPart::kControlAnimated);
	jPart->setPosition(0.0,28.0, 5.0);

	VxCollisionGeometry* jGeometry = new VxCollisionGeometry( new VxBox(1.0, 1.0, 1.0) );
	jPart->addCollisionGeometry(jGeometry);

	jMechanism->addAssembly(jAssembly);

	scene->add(jMechanism);
/*************/



#ifdef USE_OSG
    // Create a camera for the scene.
    Vx::VxSmartPtr<VxSim::VxExtension> freeCameraExtension = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::PerspectiveICD::kExtensionFactoryKey);
    VxGraphics::ICamera* freeCamera = VxGraphics::ICamera::getInterface(freeCameraExtension.get());

    // Position the camera in the world.
    Vx::VxTransform tm = Vx::VxTransform(Vx::VxVector3(-50,-35, 35), Vx::VxEulerAngles(VX_DEG2RAD(-15), VX_DEG2RAD(25), VX_DEG2RAD(40)));
    freeCamera->setTransform(tm);

    // Add a camera to the scene.
    scene->add(freeCameraExtension.get());
#endif

    return scene;
}


// Create the load mechanism and return it.
// The caller owns the pointer.
VxSim::VxMechanism* ExCableSystem::_createLoad()
{
    // Create the mechanism.
    VxSim::VxMechanism* mechanism = new VxSim::VxMechanism();

    mechanism->addAssembly(_createLoadAssembly());

    return mechanism;
}


// Create the ground mechanism and return it.
// The caller owns the pointer.
VxSim::VxMechanism* ExCableSystem::_createGround()
{
    // Create the mechanism.
    VxSim::VxMechanism *groundMechanism = new VxSim::VxMechanism();

    groundMechanism->addAssembly(_createGroundAssembly());

    return groundMechanism;
}

// Create the ground assembly and return it.
// The caller owns the pointer.
VxAssembly* ExCableSystem::_createGroundAssembly()
{
    VxAssembly* groundAssembly = new VxAssembly();
    groundAssembly->setName("groundAssembly");

    // The ground is static. It behaves as having infinite mass.
    // Therefore, you don't need to set the ground's mass.
    VxPart* groundPart = new VxPart();
    groundAssembly->addPart(groundPart);

    // It is set as static since the ground must not move when force is applied to it.
    // It supports the crane and the load; it does not fall under gravity.
    groundPart->setControl(Vx::VxPart::kControlStatic);
    // It is not required to set the name of the part, but it makes it easier to find the ground part in the debugger.
    groundPart->setName("groundPart");
    // Center the ground at (0,0) and set the ground to be at z=-0.1; i.e., the top of the collision geometry to be at z=0.
    groundPart->setPosition(0.0, 0.0, -0.1);
    // Make the box big enough to have space around the crane.
    groundPart->addCollisionGeometry(new VxCollisionGeometry(new VxBox(100.0, 100.0, 0.2)));

    return groundAssembly;
}


// Create the load assembly and return it.
// The caller owns the pointer.
VxAssembly* ExCableSystem::_createLoadAssembly()
{
    VxAssembly* loadAssembly = new VxAssembly();
    loadAssembly->setName(sLoadAssemblyName.c_str());

    const VxReal mass = 400.0;
    VxPart* loadPart = new VxPart(mass);
    loadAssembly->addPart(loadPart);

    // The load should move by the forces applied to it; hence, it must be dynamic.
    loadPart->setControl(Vx::VxPart::kControlDynamic);
    // It is not required to set the name of the part, but it makes it easier to find the ground part in the debugger.
    loadPart->setName(sLoadName.c_str());
    loadPart->setPosition(0.0, 28.0, 0.5);

    VxCollisionGeometry* collisionGeometry = new VxCollisionGeometry(new VxBox(1.0, 1.0, 1.0));
    loadPart->addCollisionGeometry(collisionGeometry);

    return loadAssembly;
}

// Create the cable system and set its definition to behave correctly for
// the crane and load mechanisms.
void ExCableSystem::_createCableSystemForCrane()
{

    VxSim::VxMechanism* craneMechanism = mCrane->getMechanism();
    VxAssembly* craneAssembly = GetAssemblyInMechanismFromName(craneMechanism, MyCrane::sCraneAssemblyName);
    if ( NULL == craneAssembly )
    {
        VxWarning(0, "The crane assembly was not found in the crane mechanism.\n");
        return;
    }

    VxSim::VxExtension* cableSystemExtension =  VxSim::VxExtensionFactory::create(CableSystemsICD::Extensions::kDynamicsKey);
    cableSystemExtension->setName(sMyDynamicsExtensionName.c_str());
    // Since the crane use the cable system, it makes sense to add it
    // to the crane mechanism.
    craneMechanism->add(cableSystemExtension);

    // The cable system always has a definition. Retrieve it to fill it
    // with the good definitions.
    VxData::Container& container = cableSystemExtension->getParameterContainer();
    VxData::FieldBase& defFieldBase = container[kDefinitionID];
    VxData::Container& definition = dynamic_cast<VxData::Container&>(defFieldBase);
    VxData::FieldBase& fieldBasePoints = definition[CableSystemDefinitionContainerID::kPointDefinitionsID];
//    if ( ! fieldBasePoints["size"].setValue(static_cast<unsigned int>(4)) ) // We will set 4 items below
    if ( ! fieldBasePoints["size"].setValue(static_cast<unsigned int>(5)) ) // We will set 5 items below
    {
        VxFatalError(0, "Cannot resize the List of PointDefinition\n");
    }


    // The cable system starts at the winch pass over the mid pulley, then the tip pulley and ends at the load.
    // Create the point definition for each contact of the cable with a part.
    VxData::Container* winch = dynamic_cast<VxData::Container*>(&(fieldBasePoints["0"]));

    (*winch)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kWinch));
    (*winch)[PointDefinitionContainerID::kVxPartID].setValue(Vx::Find::part(MyCrane::sWinchName, craneAssembly));

    // IMPORTANT: The mid point must be added in the right order.
    VxData::Container& midPulley = dynamic_cast<VxData::Container&>(fieldBasePoints["1"]);
    midPulley[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kPulley));
    midPulley[PointDefinitionContainerID::kVxPartID].setValue(Vx::Find::part(MyCrane::sMidPulleyName, craneAssembly));
    // In some cases, CableSystems might not be able to correctly deduce on which side the cable passes.
    // This usually is the case for the winch because it has only one point to use for its deduction.
    // After launching the application for the first time, or stepping through the debugger, it is easy to spot this problem.
    // Sometimes, you can help CableSystems by inverting the guess with:
    if ( !midPulley[PulleyDefinitionContainerID::kInverseWrappingID].setValue(true) )
    {
        VxInfo(0, "Cannot set the value of the inverse wrapping in the mid pulley\n");
    }

    VxData::Container& tipPulley = dynamic_cast<VxData::Container&>(fieldBasePoints["2"]);
    tipPulley[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kPulley));
    tipPulley[PointDefinitionContainerID::kVxPartID].setValue(Vx::Find::part(MyCrane::sTipPulleyName, craneAssembly));

	VxData::Container& ringHook = dynamic_cast<VxData::Container&>(fieldBasePoints["3"]);
	ringHook[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kRing));
    ringHook[PointDefinitionContainerID::kVxPartID].setValue(_jTestPart);
	ringHook[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(VxVector3(1, 0, 0));

    VxAssembly* loadAssembly = _getLoadAssembly();
    VxPart* load = Vx::Find::part(sLoadName, loadAssembly);
    VX_ASSERT(NULL != load, "A part named \"load\" must be in the load assembly.\n");
    const VxVector3 offset(0,0,-0.5); // Needed to attach on the top of the load and not at the center of mass.
    //VxData::Container& attachmentPoint = dynamic_cast<VxData::Container&>(fieldBasePoints["3"]);
	VxData::Container& attachmentPoint = dynamic_cast<VxData::Container&>(fieldBasePoints["4"]);
    attachmentPoint[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
    attachmentPoint[PointDefinitionContainerID::kVxPartID].setValue(load);
    attachmentPoint[PointDefinitionContainerID::kOffsetID].setValue(offset);

    // Get the last segment and change its parameters
    // Segments: "0" Arc on winch, "1" Segment between winch and midPulley, "2" Arc on midPulley
    //           "3" Segment between midPulley and tipPulley, "4" Arc on tipPulley,
    //           "5" Segment between tipPulley and Load
    VxData::FieldBase& fieldBaseSegments = definition[CableSystemDefinitionContainerID::kSegmentDefinitionsID];
    VxData::Container& lastSegmentDefinition = dynamic_cast<VxData::Container&>(fieldBaseSegments["5"]);
    lastSegmentDefinition[SegmentDefinitionContainerID::kFlexibleID].setValue(true);
    lastSegmentDefinition[SegmentDefinitionContainerID::kMaxSectionLengthID].setValue(1.0);
    lastSegmentDefinition[SegmentDefinitionContainerID::kMinSectionLengthID].setValue(0.2);
	//lastSegmentDefinition[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(2);

    VxData::Container& lastSegmentDefinition2 = dynamic_cast<VxData::Container&>(fieldBaseSegments["6"]);
    lastSegmentDefinition2[SegmentDefinitionContainerID::kFlexibleID].setValue(true);
    lastSegmentDefinition2[SegmentDefinitionContainerID::kMaxSectionLengthID].setValue(3.0);
    lastSegmentDefinition2[SegmentDefinitionContainerID::kMinSectionLengthID].setValue(0.2);
	lastSegmentDefinition2[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(2);

    VxData::FieldBase& fieldBaseParams = definition[CableSystemDefinitionContainerID::kParamDefinitionID];
    VxData::Container& params = dynamic_cast<VxData::Container&>(fieldBaseParams);
    params[CableSystemParamDefinitionContainerID::kAxialStiffnessID].setValue(10000.0);
    params[CableSystemParamDefinitionContainerID::kAxialDampingID].setValue(2000.0);
	//params[CableSystemParamDefinitionContainerID::kCollisionGeometryTypeID].setValue(2);


}

// Should always return a valid assembly.
// The caller owns the returned pointer.
VxAssembly* ExCableSystem::_getLoadAssembly()
{
    return GetAssemblyInMechanismFromName(mLoadMechanism.get(), sLoadAssemblyName);
}
