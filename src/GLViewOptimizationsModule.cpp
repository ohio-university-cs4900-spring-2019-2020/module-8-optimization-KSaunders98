#include "GLViewOptimizationsModule.h"

#include <chrono>

#include "Axes.h" //We can set Axes to on/off with this
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "PhysicsEngineODE.h"
#include "WorldList.h" //This is where we place all of our WOs

//Different WO used by this module
#include "AftrGLRendererBase.h"
#include "AftrGeometryFrustum.h"
#include "Camera.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "CameraChaseActorSmooth.h"
#include "CameraStandard.h"
#include "MGLFrustum.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WO.h"
#include "WOCar1970sBeater.h"
#include "WOHumanCal3DPaladin.h"
#include "WOHumanCyborg.h"
#include "WOLight.h"
#include "WONVDynSphere.h"
#include "WONVPhysX.h"
#include "WONVStaticPlane.h"
#include "WOSkyBox.h"
#include "WOStatic.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOWayPointSpherical.h"

using namespace Aftr;

// frustum constants
const float NEAR = 5.0f;
const float FAR = 30.0f;
const float ASPECT = 2.0f;
const float FOV_H = 45.0f;

GLViewOptimizationsModule* GLViewOptimizationsModule::New(const std::vector<std::string>& args)
{
    GLViewOptimizationsModule* glv = new GLViewOptimizationsModule(args);
    glv->init(Aftr::GRAVITY, Vector(0, 0, -1.0f), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE);
    glv->onCreate();
    return glv;
}

GLViewOptimizationsModule::GLViewOptimizationsModule(const std::vector<std::string>& args)
    : GLView(args)
{
    //Initialize any member variables that need to be used inside of LoadMap() here.
    //Note: At this point, the Managers are not yet initialized. The Engine initialization
    //occurs immediately after this method returns (see GLViewOptimizationsModule::New() for
    //reference). Then the engine invoke's GLView::loadMap() for this module.
    //After loadMap() returns, GLView::onCreate is finally invoked.

    //The order of execution of a module startup:
    //GLView::New() is invoked:
    //    calls GLView::init()
    //       calls GLView::loadMap() (as well as initializing the engine's Managers)
    //    calls GLView::onCreate()

    //GLViewOptimizationsModule::onCreate() is invoked after this module's LoadMap() is completed.
    rotate = false;
}

void GLViewOptimizationsModule::onCreate()
{
    //GLViewOptimizationsModule::onCreate() is invoked after this module's LoadMap() is completed.
    //At this point, all the managers are initialized. That is, the engine is fully initialized.

    if (this->pe != NULL) {
        //optionally, change gravity direction and magnitude here
        //The user could load these values from the module's aftr.conf
        this->pe->setGravityNormalizedVector(Vector(0, 0, -1.0f));
        this->pe->setGravityScalar(Aftr::GRAVITY);
    }
    this->setActorChaseType(STANDARDEZNAV); //Default is STANDARDEZNAV mode
    //this->setNumPhysicsStepsPerRender( 0 ); //pause physics engine on start up; will remain paused till set to 1
}

GLViewOptimizationsModule::~GLViewOptimizationsModule()
{
    //Implicitly calls GLView::~GLView()
}

void GLViewOptimizationsModule::updateWorld()
{
    GLView::updateWorld(); //Just call the parent's update world first.
        //If you want to add additional functionality, do it after
        //this call.

    using namespace std::chrono;

    // calculate delta time
    static auto last_time = steady_clock::now();
    auto now = steady_clock::now();
    float dt = duration_cast<duration<float>>(now - last_time).count();
    last_time = now;

    if (rotate) {
        Mat4 m = frustum->getDisplayMatrix();
        m = m.rotate(Vector(0.0f, 0.0f, 1.0f), dt * Aftr::PI / 4.0f);
        frustum->getModel()->setDisplayMatrix(m);
    }

    for (size_t i = 0; i < teapots.size(); ++i) {
        if (isVisible(teapots[i])) {
            teapots[i]->isVisible = true;
        } else {
            teapots[i]->isVisible = false;
        }

        ModelMeshSkin& skin = teapots[i]->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);
        if (isVisibleToFrustum(teapots[i])) {
            skin.setDiffuse(aftrColor4f(0.0f, 0.0f, 1.0f, 1.0f));
        } else {
            skin.setDiffuse(aftrColor4f(1.0f, 0.0f, 0.0f, 1.0f));
        }
    }
}

void GLViewOptimizationsModule::onResizeWindow(GLsizei width, GLsizei height)
{
    GLView::onResizeWindow(width, height); //call parent's resize method.
}

void GLViewOptimizationsModule::onMouseDown(const SDL_MouseButtonEvent& e)
{
    GLView::onMouseDown(e);
}

void GLViewOptimizationsModule::onMouseUp(const SDL_MouseButtonEvent& e)
{
    GLView::onMouseUp(e);
}

void GLViewOptimizationsModule::onMouseMove(const SDL_MouseMotionEvent& e)
{
    GLView::onMouseMove(e);
}

void GLViewOptimizationsModule::onKeyDown(const SDL_KeyboardEvent& key)
{
    GLView::onKeyDown(key);

    if (key.keysym.sym == SDLK_l) {
        std::cout << "=====================================================" << std::endl;
        std::cout << "Visible Teapots: ";
        for (size_t i = 0; i < teapots.size(); ++i) {
            if (teapots[i]->isVisible) {
                std::cout << "teapot" << i << " ";
            }
        }
        std::cout << std::endl;
    } else if (key.keysym.sym == SDLK_r) {
        rotate = !rotate;
    }
}

void GLViewOptimizationsModule::onKeyUp(const SDL_KeyboardEvent& key)
{
    GLView::onKeyUp(key);
}

void Aftr::GLViewOptimizationsModule::loadMap()
{
    this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
    this->actorLst = new WorldList();
    this->netLst = new WorldList();

    ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
    ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
    ManagerOpenGLState::enableFrustumCulling = false;
    Axes::isVisible = false;
    this->glRenderer->isUsingShadowMapping(false); //set to TRUE to enable shadow mapping, must be using GL 3.2+

    this->cam->setPosition(15, 15, 10);
    this->cam->setCameraLookAtPoint(Vector(0, 0, 0));

    std::string grass(ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl");
    std::string teapotModel(ManagerEnvironmentConfiguration::getLMM() + "/models/teapot.obj");

    //SkyBox Textures readily available
    std::vector<std::string> skyBoxImageNames; //vector to store texture paths
    skyBoxImageNames.push_back(ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_mountains+6.jpg");

    float ga = 0.1f; //Global Ambient Light level for this module
    ManagerLight::setGlobalAmbientLight(aftrColor4f(ga, ga, ga, 1.0f));
    WOLight* light = WOLight::New();
    light->isDirectionalLight(true);
    light->setPosition(Vector(0, 0, 100));
    //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
    //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
    light->getModel()->setDisplayMatrix(Mat4::rotateIdentityMat({ 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD));
    light->setLabel("Light");
    worldLst->push_back(light);

    //Create the SkyBox
    WO* wo = WOSkyBox::New(skyBoxImageNames.at(0), this->getCameraPtrPtr());
    wo->setPosition(Vector(0, 0, 0));
    wo->setLabel("Sky Box");
    wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    worldLst->push_back(wo);

    ////Create the infinite grass plane (the floor)
    wo = WO::New(grass, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
    wo->setPosition(Vector(0, 0, 0));
    wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    ModelMeshSkin& grassSkin = wo->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);
    grassSkin.getMultiTextureSet().at(0)->setTextureRepeats(5.0f);
    grassSkin.setAmbient(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Color of object when it is not in any light
    grassSkin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f)); //Diffuse color components (ie, matte shading color of this object)
    grassSkin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
    grassSkin.setSpecularCoefficient(10); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
    wo->setLabel("Grass");
    worldLst->push_back(wo);

    for (int i = -25; i <= 25; ++i) {
        WO* teapot = WO::New(teapotModel, Vector(2.0f, 2.0f, 2.0f), MESH_SHADING_TYPE::mstFLAT);
        teapot->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        teapot->setPosition(cos(static_cast<float>(i) * Aftr::PI / 4.0f) * 10.0f, static_cast<float>(i) * 5.0f, 15.0f + cos(static_cast<float>(i) * Aftr::PI / 4.0f) * 5.0f);
        teapot->getModel()->renderBBox = true;
        worldLst->push_back(teapot);
        teapots.push_back(teapot);
    }

    frustum = WO::New();
    frustum->setModel(MGLFrustum::New(frustum, NEAR, FAR, FOV_H, ASPECT));
    frustum->setPosition(-10.0f, 0, 14.0f);
    worldLst->push_back(frustum);
}

bool GLViewOptimizationsModule::isInFrustum(WO* wo, const AftrGeometryFrustum& frust) const
{
    Mat4 mat = wo->getDisplayMatrix();
    Vector position = wo->getPosition();
    Vector boxHalfExtents = wo->getModel()->getBoundingBox().getlxlylz() * 0.5f;
    for (unsigned int plane = 0; plane < 6; ++plane) {
        Vector normal = frust.getPlaneNormal(plane);
        float coeff = frust.getPlaneCoef(plane);

        // check if each vertex of the bbox is outside this frustum plane
        bool outside = true;
        for (int x = -1; x <= 1; x += 2) {
            for (int y = -1; y <= 1; y += 2) {
                for (int z = -1; z <= 1; z += 2) {
                    Vector v = mat * (Vector(x, y, z) * boxHalfExtents) + position;
                    if (normal.dotProduct(v) < coeff) {
                        outside = false;
                    }
                }
            }
        }

        if (outside) {
            return false;
        }
    }

    return true;
}

bool GLViewOptimizationsModule::isVisible(WO* wo) const
{
    float near = this->cam->getCameraNearClippingPlaneDistance();
    float far = this->cam->getCameraFarClippingPlaneDistance();
    float aspect = this->cam->getCameraAspectRatioWidthToHeight();
    float fov_h = this->cam->getCameraHorizontalFOVDeg();
    float fov_v = 2 * atan(tan(fov_h * Aftr::DEGtoRAD * .5f) / aspect) * Aftr::RADtoDEG;
    Vector look = this->cam->getLookDirection();
    Vector norm = this->cam->getNormalDirection();
    Vector pos = this->cam->getPosition();

    AftrGeometryFrustum frust(aspect, fov_v, near, far, look, norm, pos);

    return isInFrustum(wo, frust);
}

bool GLViewOptimizationsModule::isVisibleToFrustum(WO* wo) const
{
    MGLFrustum* frustumM = frustum->getModelT<MGLFrustum>();
    float near = frustumM->getNearPlane();
    float far = frustumM->getFarPlane();
    float aspect = frustumM->getAspectRatioWidthToHeight();
    float fov_h = frustumM->getHorzFOVDeg();
    float fov_v = 2 * atan(tan(fov_h * Aftr::DEGtoRAD * .5f) / aspect) * Aftr::RADtoDEG;
    Vector look = frustum->getLookDirection();
    Vector norm = frustum->getNormalDirection();
    Vector pos = frustum->getPosition();

    AftrGeometryFrustum frust(aspect, fov_v, near, far, look, norm, pos);

    return isInFrustum(wo, frust);
}
