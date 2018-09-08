#ifndef FISH_UI_VIEWER_PLUGIN_H
#define FISH_UI_VIEWER_PLUGIN_H

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>
#include <igl/igl_inline.h>

struct ImGuiContext;

class FishUIViewerPlugin : public igl::opengl::glfw::ViewerPlugin {
public:
    IGL_INLINE virtual void init(igl::opengl::glfw::Viewer* _viewer) override;
    IGL_INLINE virtual void shutdown() override;

    IGL_INLINE virtual void reload_font(int font_size = 14);

    IGL_INLINE virtual bool pre_draw() override;
    IGL_INLINE  virtual bool post_draw() override;

    IGL_INLINE virtual void post_resize(int width, int height) override;

    // Mouse IO
    IGL_INLINE virtual bool mouse_down(int button, int modifier) override;
    IGL_INLINE virtual bool mouse_up(int button, int modifier) override;
    IGL_INLINE virtual bool mouse_move(int mouse_x, int mouse_y) override;
    IGL_INLINE virtual bool mouse_scroll(float delta_y) override;

    // Keyboard IO
    IGL_INLINE virtual bool key_pressed(unsigned int key, int modifiers) override;
    IGL_INLINE virtual bool key_down(int key, int modifiers) override;
    IGL_INLINE virtual bool key_up(int key, int modifiers) override;

    IGL_INLINE void draw_labels_window();
    IGL_INLINE void draw_labels(const igl::opengl::ViewerData& data);
    IGL_INLINE void draw_text(Eigen::Vector3d pos, Eigen::Vector3d normal,
        const std::string& text);

    IGL_INLINE float pixel_ratio();
    IGL_INLINE float hidpi_scaling();
    float menu_scaling() { return hidpi_scaling_ / pixel_ratio_; }

protected:
    // Hidpi scaling to be used for text rendering.
    float hidpi_scaling_;

    // Ratio between the framebuffer size and the window size.
    // May be different from the hipdi scaling!
    float pixel_ratio_;

    // ImGui Context
    ImGuiContext* context_ = nullptr;
};

#endif // FISH_UI_VIEWER_PLUGIN_H
