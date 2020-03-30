#pragma once

#include "GLView.h"

namespace Aftr {
class Camera;
class AftrGeometryFrustum;

/**
   \class GLViewOptimizationsModule
   \author Scott Nykl 
   \brief A child of an abstract GLView. This class is the top-most manager of the module.

   Read \see GLView for important constructor and init information.

   \see GLView

    \{
*/

class GLViewOptimizationsModule : public GLView {
public:
    static GLViewOptimizationsModule* New(const std::vector<std::string>& outArgs);
    virtual ~GLViewOptimizationsModule();
    virtual void updateWorld(); ///< Called once per frame
    virtual void loadMap(); ///< Called once at startup to build this module's scene
    virtual void onResizeWindow(GLsizei width, GLsizei height);
    virtual void onMouseDown(const SDL_MouseButtonEvent& e);
    virtual void onMouseUp(const SDL_MouseButtonEvent& e);
    virtual void onMouseMove(const SDL_MouseMotionEvent& e);
    virtual void onKeyDown(const SDL_KeyboardEvent& key);
    virtual void onKeyUp(const SDL_KeyboardEvent& key);

protected:
    GLViewOptimizationsModule(const std::vector<std::string>& args);
    virtual void onCreate();

    bool isInFrustum(WO* wo, const AftrGeometryFrustum& frust) const;
    bool isVisible(WO* wo) const;
    bool isVisibleToFrustum(WO* wo) const;

    WO* frustum;
    std::vector<WO*> teapots;
    bool rotate;
};

/** \} */

} //namespace Aftr
