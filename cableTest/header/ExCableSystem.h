#ifndef _EX_CABLE_SYSTEM_H
#define _EX_CABLE_SYSTEM_H

#include <VxSim/VxScene.h>

#include <Vx/VxSmartPtr.h>

// Forward Declaration
class MyCrane;

namespace Vx
{
    class VxHinge;
    class VxPart;
}

// Module tutorial setup class to be used with the MechanismViewer.
// The module creates the scene used to populate the tutorial.
// This is equivalent to load a VXS scene from file.
class ExCableSystem
{
public:

    // Constructor
    //
    ExCableSystem();

    // Destructor
    //
    virtual ~ExCableSystem();

    // Called to get the scene
    //
    VxSim::VxScene* getScene();

private:
    // @internal helpers
    VxSim::VxScene* _createScene();
    VxSim::VxMechanism* _createCrane();
    VxSim::VxMechanism* _createLoad();
    VxSim::VxMechanism* _createGround();

    Vx::VxAssembly* _createLoadAssembly();
    Vx::VxAssembly* _createGroundAssembly();

    Vx::VxAssembly* _getLoadAssembly();

    void _createCableSystemForCrane();

private:

	Vx::VxPart* _jTestPart;

    // My reference to the scene
    Vx::VxSmartPtr<VxSim::VxScene> mScene;

    // Pointers to the concrete (objects) in order to modify their behavior during onPreUpdate()
    MyCrane* mCrane;

    // The load is another mechanism to be able to have collision between the crane and the load
    Vx::VxSmartPtr<VxSim::VxMechanism> mLoadMechanism;
};

#endif // _EX_CABLE_SYSTEM_H
