#include "KeyboardExtension.h"
#include "ExCableSystem.h"

#include <CableSystems/CableSystemsICD.h>
#include <CableSystems/DynamicsICD.h>
#include <CableSystems/GraphicsICD.h>

#ifdef USE_OSG
#include <VxGraphics/GraphicsModule.h>
#include <VxGraphics/ICamera.h>
#include <VxGraphicsPlugins/PerspectiveICD.h>
#endif

#include <VxSim/VxApplication.h>
#include <VxSim/VxMechanism.h>
#include <VxSim/VxSimulatorModuleFactory.h>
#include <VxSim/VxExtensionFactory.h>
#include <VxSim/VxDynamicsModuleICD.h>

#include <VxData/FieldArray.h>
#include <VxData/Vector.h>

#include <Vx/VxAssembly.h>
#include <Vx/VxBox.h>
#include <Vx/VxCollisionGeometry.h>
#include <Vx/VxHinge.h>
#include <Vx/VxMaterial.h>
#include <Vx/VxContactMaterial.h>
#include <Vx/VxPart.h>
#include <Vx/VxPlane.h>
#include <Vx/VxTransform.h>

#include <Vx/VxSmartPtr.h>

#include <VxPluginSystem/VxPluginManager.h>
#include <Vx/VxConnectionFactory.h>

#ifdef USE_OSG
#include <VxGraphicsPlugins/DynamicsVisualizerICD.h>
#include <VxGraphicsPlugins/GraphicsModuleICD_OSG.h>
#endif

#include <iostream>
using std::cout;
using std::endl;

int main (int argc, const char * argv[])
{

	int returnValue = 0;
    
	try
    {
		// Instantiate the Vortex application.
        Vx::VxSmartPtr<VxSim::VxApplication> application = new VxSim::VxApplication;

#ifdef USE_OSG
        // Instantiate a Graphic module using OSG and add it to the application.
        VxPluginSystem::VxPluginManager::instance()->load("VxGraphicsModuleOSG");
        Vx::VxSmartPtr<VxSim::VxSimulatorModule> graphicsSimulatorModule = VxSim::VxSimulatorModuleFactory::create(VxGraphicsPlugins::GraphicsModuleICD::kModuleFactoryKey);
        application->insertModule(graphicsSimulatorModule.get());
        // The VxGraphicsModuleInterface gives access to the VxGraphicsModule interface inside the module.
        VxGraphics::GraphicsModule* graphicModule= VxGraphics::GraphicsModule::getInterface(graphicsSimulatorModule.get());
        graphicModule->createDefaultWindow();

        // Create a default camera for the Graphic module.
        Vx::VxSmartPtr<VxSim::VxExtension> freeCameraExtension = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::PerspectiveICD::kExtensionFactoryKey);
        VxGraphics::ICamera* freeCamera = VxGraphics::ICamera::getInterface(freeCameraExtension.get());

        // Position the camera in the world.
        Vx::VxTransform tm = Vx::VxTransform(Vx::VxVector3(35, 5, 10), Vx::VxEulerAngles(VX_DEG2RAD(90), VX_DEG2RAD(90), VX_DEG2RAD(0)));
        freeCamera->setTransform(tm);

        graphicsSimulatorModule->addExtension(freeCameraExtension.get());

        // Set the free camera as the active camera using the interface specialized for the VxGraphicModule.
        assert( NULL != graphicModule );

        graphicModule->setCamera(freeCamera);
#endif

        // Create a material.
        Vx::VxMaterial *groundMat = new Vx::VxMaterial;
        const Vx::VxReal defaultSlip = 1e-4;
        groundMat->setFrictionModel(Vx::VxContactMaterial::kFrictionAxisAngularNormal, Vx::VxContactMaterial::kFrictionModelNeutral);
        groundMat->setFrictionModel(Vx::VxContactMaterial::kFrictionAxisAngularPrimary, Vx::VxContactMaterial::kFrictionModelNeutral);
        groundMat->setFrictionModel(Vx::VxContactMaterial::kFrictionAxisAngularSecondary, Vx::VxContactMaterial::kFrictionModelNeutral);
        groundMat->setFrictionModel(Vx::VxContactMaterial::kFrictionAxisLinear, Vx::VxContactMaterial::kFrictionModelScaledBox);
        groundMat->setFrictionCoefficient(Vx::VxContactMaterial::kFrictionAxisAngularPrimary, 1.0f);
        groundMat->setFrictionCoefficient(Vx::VxContactMaterial::kFrictionAxisAngularSecondary, 1.0f);
        groundMat->setFrictionCoefficient(Vx::VxContactMaterial::kFrictionAxisLinear, 1.0f);
        groundMat->setSlip(Vx::VxContactMaterial::kFrictionAxisLinear, defaultSlip);
        groundMat->setCompliance(1.0e-6f);
        groundMat->setDamping(1.0e005f);
        groundMat->setIntegratedSlipDisplacement(Vx::VxMaterial::kIntegratedSlipDisplacementActivated);
        groundMat->setName("concrete");

        // Add a plane, so that the crane has something to run on.
        Vx::VxPart* part = new Vx::VxPart();
        part->setTransform(Vx::VxTransform::createIdentity());
		Vx::VxCollisionGeometry* groundGeom = new Vx::VxCollisionGeometry(new Vx::VxPlane(), groundMat);
        part->addCollisionGeometry(groundGeom);
        part->setControl(Vx::VxPart::kControlStatic);

        Vx::VxAssembly* ground = new Vx::VxAssembly;
        ground->addPart(part);
        Vx::VxSmartPtr<VxSim::VxMechanism> groundMechanism = new VxSim::VxMechanism;
        groundMechanism.get()->addAssembly(ground);

        // Instantiate a Framework dynamics module add it to the application.
        Vx::VxSmartPtr<VxSim::VxSimulatorModule> dynamicsModule = VxSim::VxSimulatorModuleFactory::create(VxSim::VxDynamicsModuleICD::kFactoryKey);
        application->insertModule(dynamicsModule.get());

		/*
        // Create the crane with the CableSystems and add it to the scene.
        // Here, a cable system is created.  
        // Note that we could instead load an existing mechanism file with an existing CableSystems 
        ExCableSystem cableSystem;
        VxSim::VxScene* scene = cableSystem.getScene();
        scene->add(groundMechanism.get());

        // Find the first CableSystem dynamics extension.
        Vx::VxSmartPtr<VxSim::VxExtension> cableExtension;
        size_t mechanismCount = scene->getSimObjectCount();
        for (size_t mechanismIndex = 0; mechanismIndex < mechanismCount; ++mechanismIndex)
        {
            Vx::VxSmartPtr<VxSim::VxSimObject> mechanism = scene->getSimObject(mechanismIndex);
            if (mechanism.valid())
            {
                std::cout << "****************************" << std::endl << "- Mechanism: " << mechanism->getName() << std::endl;

                size_t extCount = mechanism->getExtensionCount();
                for (size_t extIndex = 0; extIndex < extCount; ++extIndex)
                {
                    Vx::VxSmartPtr<VxSim::VxExtension> ext = mechanism->getExtension(extIndex);
                    VxSim::VxPluginExtension* pluginExt = dynamic_cast<VxSim::VxPluginExtension*>(ext.get());
                    if (pluginExt) 
                    {                    
                        if (pluginExt->getCreationKey() == CableSystemsICD::Extensions::kDynamicsKey)
                        {
                            cableExtension = ext;
                            break;
                        }
                    }
                }
            }
        }
		*/

	/** Alex's test **/
    VxSim::VxMechanism* mechanism = new VxSim::VxMechanism();
    mechanism->setName("cableTestMechanism");

	Vx::VxAssembly* assembly = new Vx::VxAssembly();
    assembly->setName("cableTestAssembly");

	Vx::VxPart* bPart = new Vx::VxPart();
	bPart->setName("testBrick");
	bPart->setPosition(0, 20.0, 3.0);
	Vx::VxCollisionGeometry* ge = new Vx::VxCollisionGeometry( new Vx::VxBox(0.5, 10.0, 0.5));
	bPart->addCollisionGeometry(ge);
	bPart->setControl(Vx::VxPart::kControlAnimated);
	assembly->addPart(bPart);

	Vx::VxPart*	brick = new Vx::VxPart();
	Vx::VxPart* brick2 = brick;
	brick->setName("brick_1");
	brick->setPosition(5.0, 20.0, 10.0);
	Vx::VxCollisionGeometry* geom = new Vx::VxCollisionGeometry(new Vx::VxBox(1.0, 1.0, 1.0));
	brick->addCollisionGeometry(geom);
	brick->setControl(Vx::VxPart::kControlDynamic);
//	brick->setControl(Vx::VxPart::kControlAnimated);
	assembly->addPart(brick);

	brick = new Vx::VxPart();
	Vx::VxPart* brick3 = brick;
	brick->setName("brick_3");
	brick->setPosition(0.0, 30.0, 5.0);
	geom = new Vx::VxCollisionGeometry(new Vx::VxBox(1.0, 1.0, 1.0));
	brick->addCollisionGeometry(geom);
//	brick->setControl(Vx::VxPart::kControlDynamic);
	brick->setControl(Vx::VxPart::kControlAnimated);
	assembly->addPart(brick);

	brick = new Vx::VxPart();
	Vx::VxPart* brick4 = brick;
	brick->setName("brick_4");
	brick->setPosition(0.0, 10.0, 5.0);
	geom = new Vx::VxCollisionGeometry(new Vx::VxBox(1.0, 1.0, 1.0));
	brick->addCollisionGeometry(geom);
//	brick->setControl(Vx::VxPart::kControlDynamic);
	brick->setControl(Vx::VxPart::kControlAnimated);
	assembly->addPart(brick);

	brick = new Vx::VxPart();
	brick->setName("brick_2");
	brick->setPosition(-5.0, 20.0, 10.0);
	geom = new Vx::VxCollisionGeometry(new Vx::VxBox(1.0, 1.0, 1.0));
	brick->addCollisionGeometry(geom);
	brick->setControl(Vx::VxPart::kControlDynamic);
//	brick->setControl(Vx::VxPart::kControlAnimated);
	assembly->addPart(brick);

	mechanism->addAssembly(assembly);
	/*
	VxSim::VxExtension* cable = VxSim::VxExtensionFactory::create(CableSystemsICD::Extensions::kDynamicsKey);
	cable->setName("testCableExtension");
	mechanism->add(cable);

	using namespace Vx;
	using namespace CableSystems;
	using namespace CableSystems::DynamicsICD;

	VxData::Container& container = cable->getParameterContainer();
    VxData::FieldBase& defFieldBase = container[kDefinitionID];
    VxData::Container& definition = dynamic_cast<VxData::Container&>(defFieldBase);
    VxData::FieldBase& fieldBasePoints = definition[CableSystemDefinitionContainerID::kPointDefinitionsID];
    if ( ! fieldBasePoints["size"].setValue(static_cast<unsigned int>(4)) ) // We will set 5 items below
    {
        VxFatalError(0, "Cannot resize the List of PointDefinition\n");
    }
	
	const Vx::VxVector3 offset(0.5,0,0); // Needed to attach on the top of the load and not at the center of mass.
    //VxData::Container& attachmentPoint = dynamic_cast<VxData::Container&>(fieldBasePoints["3"]);
	VxData::Container& attachmentPoint = dynamic_cast<VxData::Container&>(fieldBasePoints["0"]);
    attachmentPoint[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
    attachmentPoint[PointDefinitionContainerID::kVxPartID].setValue(brick);
    attachmentPoint[PointDefinitionContainerID::kOffsetID].setValue(offset);

    //VxData::Container& attachmentPoint = dynamic_cast<VxData::Container&>(fieldBasePoints["3"]);
	VxData::Container& attachmentPoint2 = dynamic_cast<VxData::Container&>(fieldBasePoints["1"]);
    attachmentPoint2[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
    attachmentPoint2[PointDefinitionContainerID::kVxPartID].setValue(brick2);
    attachmentPoint2[PointDefinitionContainerID::kOffsetID].setValue(offset);

	    //VxData::Container& attachmentPoint = dynamic_cast<VxData::Container&>(fieldBasePoints["3"]);
	VxData::Container& attachmentPoint3 = dynamic_cast<VxData::Container&>(fieldBasePoints["2"]);
    attachmentPoint3[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
    attachmentPoint3[PointDefinitionContainerID::kVxPartID].setValue(brick3);
    attachmentPoint3[PointDefinitionContainerID::kOffsetID].setValue(offset);

	    //VxData::Container& attachmentPoint = dynamic_cast<VxData::Container&>(fieldBasePoints["3"]);
	VxData::Container& attachmentPoint4 = dynamic_cast<VxData::Container&>(fieldBasePoints["3"]);
    attachmentPoint4[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
    attachmentPoint4[PointDefinitionContainerID::kVxPartID].setValue(brick4);
    attachmentPoint4[PointDefinitionContainerID::kOffsetID].setValue(offset);

	VxData::FieldBase& fieldBaseSegments = definition[CableSystemDefinitionContainerID::kSegmentDefinitionsID];
    VxData::Container& lastSegmentDefinition = dynamic_cast<VxData::Container&>(fieldBaseSegments["0"]);
    lastSegmentDefinition[SegmentDefinitionContainerID::kFlexibleID].setValue(true);
    lastSegmentDefinition[SegmentDefinitionContainerID::kMaxSectionLengthID].setValue(11.0);
    lastSegmentDefinition[SegmentDefinitionContainerID::kMinSectionLengthID].setValue(0.2);
	lastSegmentDefinition[SegmentDefinitionContainerID::kFixedLengthID].setValue(true);

	
    VxData::Container& lastSegmentDefinition2 = dynamic_cast<VxData::Container&>(fieldBaseSegments["1"]);
    lastSegmentDefinition2[SegmentDefinitionContainerID::kFlexibleID].setValue(true);
    lastSegmentDefinition2[SegmentDefinitionContainerID::kMaxSectionLengthID].setValue(11.0);
    lastSegmentDefinition2[SegmentDefinitionContainerID::kMinSectionLengthID].setValue(0.2);
	lastSegmentDefinition2[SegmentDefinitionContainerID::kFixedLengthID].setValue(true);
	
	VxData::FieldBase& fieldBaseParams = definition[CableSystemDefinitionContainerID::kParamDefinitionID];
    VxData::Container& params = dynamic_cast<VxData::Container&>(fieldBaseParams);
    params[CableSystemParamDefinitionContainerID::kAxialStiffnessID].setValue(10000.0);
    params[CableSystemParamDefinitionContainerID::kAxialDampingID].setValue(20.0);
	//params[CableSystemParamDefinitionContainerID::kCollisionGeometryTypeID].setValue( groundGeom->getCollisionGeometryID() );
	params[CableSystemParamDefinitionContainerID::kCollisionGeometryTypeID].setValue(2);
	//scene->add(mechanism);
	*/

	VxSim::VxExtension* cable = VxSim::VxExtensionFactory::create(CableSystemsICD::Extensions::kDynamicsKey);
	cable->setName("testCableExtension");
	mechanism->add(cable);

	using namespace Vx;
	using namespace CableSystems;
	using namespace CableSystems::DynamicsICD;

	VxData::Container& container = cable->getParameterContainer();
    VxData::FieldBase& defFieldBase = container[kDefinitionID];
    VxData::Container& definition = dynamic_cast<VxData::Container&>(defFieldBase);
    VxData::FieldBase& fieldBasePoints = definition[CableSystemDefinitionContainerID::kPointDefinitionsID];
    if ( ! fieldBasePoints["size"].setValue(static_cast<unsigned int>(2)) ) // We will set 5 items below
    {
        VxFatalError(0, "Cannot resize the List of PointDefinition\n");
    }
	
	const Vx::VxVector3 offset(0.5,0,0); // Needed to attach on the top of the load and not at the center of mass.
    //VxData::Container& attachmentPoint = dynamic_cast<VxData::Container&>(fieldBasePoints["3"]);
	VxData::Container& attachmentPoint = dynamic_cast<VxData::Container&>(fieldBasePoints["0"]);
    attachmentPoint[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
    attachmentPoint[PointDefinitionContainerID::kVxPartID].setValue(brick);
    attachmentPoint[PointDefinitionContainerID::kOffsetID].setValue(offset);

    //VxData::Container& attachmentPoint = dynamic_cast<VxData::Container&>(fieldBasePoints["3"]);
	VxData::Container& attachmentPoint2 = dynamic_cast<VxData::Container&>(fieldBasePoints["1"]);
    attachmentPoint2[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
    attachmentPoint2[PointDefinitionContainerID::kVxPartID].setValue(brick2);
    attachmentPoint2[PointDefinitionContainerID::kOffsetID].setValue(offset);

	VxData::FieldBase& fieldBaseSegments = definition[CableSystemDefinitionContainerID::kSegmentDefinitionsID];
    VxData::Container& lastSegmentDefinition = dynamic_cast<VxData::Container&>(fieldBaseSegments["0"]);
    lastSegmentDefinition[SegmentDefinitionContainerID::kFlexibleID].setValue(true);
    lastSegmentDefinition[SegmentDefinitionContainerID::kMaxSectionLengthID].setValue(11.0);
    lastSegmentDefinition[SegmentDefinitionContainerID::kMinSectionLengthID].setValue(0.2);
	lastSegmentDefinition[SegmentDefinitionContainerID::kFixedLengthID].setValue(true);

		
	VxData::FieldBase& fieldBaseParams = definition[CableSystemDefinitionContainerID::kParamDefinitionID];
    VxData::Container& params = dynamic_cast<VxData::Container&>(fieldBaseParams);
    params[CableSystemParamDefinitionContainerID::kAxialStiffnessID].setValue(100.0);
    params[CableSystemParamDefinitionContainerID::kAxialDampingID].setValue(20.0);
	//params[CableSystemParamDefinitionContainerID::kCollisionGeometryTypeID].setValue( groundGeom->getCollisionGeometryID() );
	params[CableSystemParamDefinitionContainerID::kCollisionGeometryTypeID].setValue(2);
	//scene->add(mechanism);

	
	
	
	VxSim::VxExtension* cable2 = VxSim::VxExtensionFactory::create(CableSystemsICD::Extensions::kDynamicsKey);
	cable2->setName("testCableExtension2");
	mechanism->add(cable2);

	VxData::Container& container2 = cable2->getParameterContainer();
    VxData::FieldBase& defFieldBase2 = container2[kDefinitionID];
    VxData::Container& definition2 = dynamic_cast<VxData::Container&>(defFieldBase2);
    VxData::FieldBase& fieldBasePoints2 = definition2[CableSystemDefinitionContainerID::kPointDefinitionsID];
    if ( ! fieldBasePoints2["size"].setValue(static_cast<unsigned int>(2)) ) // We will set 5 items below
    {
        VxFatalError(0, "Cannot resize the List of PointDefinition\n");
    }
	

	    //VxData::Container& attachmentPoint = dynamic_cast<VxData::Container&>(fieldBasePoints["3"]);
	VxData::Container& attachmentPoint3 = dynamic_cast<VxData::Container&>(fieldBasePoints2["0"]);
    attachmentPoint3[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
    attachmentPoint3[PointDefinitionContainerID::kVxPartID].setValue(brick3);
    attachmentPoint3[PointDefinitionContainerID::kOffsetID].setValue(offset);

	    //VxData::Container& attachmentPoint = dynamic_cast<VxData::Container&>(fieldBasePoints["3"]);
	VxData::Container& attachmentPoint4 = dynamic_cast<VxData::Container&>(fieldBasePoints2["1"]);
    attachmentPoint4[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
    attachmentPoint4[PointDefinitionContainerID::kVxPartID].setValue(brick4);
    attachmentPoint4[PointDefinitionContainerID::kOffsetID].setValue(offset);
		
	VxData::FieldBase& fieldBaseSegments2 = definition2[CableSystemDefinitionContainerID::kSegmentDefinitionsID];
    VxData::Container& lastSegmentDefinition2 = dynamic_cast<VxData::Container&>(fieldBaseSegments2["0"]);
    lastSegmentDefinition2[SegmentDefinitionContainerID::kFlexibleID].setValue(true);
    lastSegmentDefinition2[SegmentDefinitionContainerID::kMaxSectionLengthID].setValue(11.0);
    lastSegmentDefinition2[SegmentDefinitionContainerID::kMinSectionLengthID].setValue(0.2);
	lastSegmentDefinition2[SegmentDefinitionContainerID::kFixedLengthID].setValue(true);
	
	VxData::FieldBase& fieldBaseParams2 = definition2[CableSystemDefinitionContainerID::kParamDefinitionID];
    VxData::Container& params2 = dynamic_cast<VxData::Container&>(fieldBaseParams2);
    params2[CableSystemParamDefinitionContainerID::kAxialStiffnessID].setValue(100000.0);
    params2[CableSystemParamDefinitionContainerID::kAxialDampingID].setValue(200.0);
	//params[CableSystemParamDefinitionContainerID::kCollisionGeometryTypeID].setValue( groundGeom->getCollisionGeometryID() );
	params2[CableSystemParamDefinitionContainerID::kCollisionGeometryTypeID].setValue(2);
	params2[CableSystemParamDefinitionContainerID::kEnableBreakageID].setValue(true);
	params2[CableSystemParamDefinitionContainerID::kMaxTensionID].setValue(1000.0);
	//scene->add(mechanism);

	 
#ifdef USE_OSG
   // The cable system is displayed with a graphic extension since it is not
   // in Vortex by default, unlike the collision geometries.
   VxSim::VxExtension* gfxExtension = VxSim::VxExtensionFactory::create(CableSystems::GraphicsICD::kFactoryKey);
   VxAssert(NULL != gfxExtension, "Cannot create the CableSystem Graphic Plugin\n");
   gfxExtension->setName("cableTestGraphics");
   mechanism->add(gfxExtension);
   Vx::VxConnectionFactory::create(cable->getOutput(kCablesID), gfxExtension->getInput(kCablesID));

   VxSim::VxExtension* gfxExtension2 = VxSim::VxExtensionFactory::create(CableSystems::GraphicsICD::kFactoryKey);
   VxAssert(NULL != gfxExtension2, "Cannot create the CableSystem Graphic Plugin\n");
   gfxExtension2->setName("cableTestGraphics2");
   mechanism->add(gfxExtension2);
   Vx::VxConnectionFactory::create(cable2->getOutput(kCablesID), gfxExtension->getInput(kCablesID));


#endif


#ifdef USE_OSG     
        // Calls to create and add the dynamics visualizer is done here merely to provide
        // a visual demonstration of the physics associated with the crane.
        // Instantiate the DynamicsVisualizer to view the physics associated with various parts that have no graphics.
        VxSim::VxExtension* dynamicsVisualizer = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::DynamicsVisualizerICD::kExtensionFactoryKey);
        dynamicsVisualizer->getInput(VxGraphicsPlugins::DynamicsVisualizerICD::kDisplayCollisionGeometry)->setValue(true);
        application->add(dynamicsVisualizer);
#endif

        // Add the scene at start
        //application->add(scene);        

	
		Vx::VxSmartPtr<VxSim::VxScene> myScene;
		myScene = new VxSim::VxScene;

		myScene->add(groundMechanism.get());
		myScene->add(mechanism);
		application->add(myScene.get());
		
		// Run the simulation.
        application->beginMainLoop();

        int stepCount = 0; 
        while(application->update())
        {
        }
        application->endMainLoop();
    }
    catch(const std::exception& ex )
    {
        std::cout << "Error : " << ex.what() << std::endl;
        returnValue = 1;
    }
    catch( ... )
    {
        Vx::VxWarning(0, "Got an unhandled exception, application will exit!\n");
        returnValue = 1;
    }

    return returnValue;

}
