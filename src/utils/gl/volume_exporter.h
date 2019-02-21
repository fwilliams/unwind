#pragma once

#include <glm/glm.hpp>
#include <glad/glad.h>

#include "../bounding_cage.h"
#include "glm_conversion.h"


glm::vec4 keyframe_bounds(BoundingCage& bc);

class VolumeExporter {
    GLuint framebuffer;
    GLuint render_texture;

    GLuint empty_vao = 0;

    struct {
        GLuint program;
        GLint ll_location;
        GLint lr_location;
        GLint ul_location;
        GLint ur_location;
        GLint texture_location;
        GLint tf_location;
    } slice;

    GLsizei w = 0, h = 0, d = 0;

public:

    glm::ivec3 export_dims() const {
        return glm::ivec3(w, h, d);
    }

    const GLuint export_texture() const {
        return render_texture;
    }

    void texture_data(std::vector<float>& ret) const {
        ret.resize(w*h*d);
        glBindTexture(GL_TEXTURE_3D, render_texture);
        glGetTexImage(GL_TEXTURE_3D, 0, GL_RED, GL_FLOAT, (void*)ret.data());
    }

    void write_texture_data_to_file(std::string filename);

    void set_export_dims(GLsizei w, GLsizei h, GLsizei d);

    void init(GLsizei w, GLsizei h, GLsizei d);

    void destroy();

    void update(BoundingCage& cage, GLuint volume_texture, glm::ivec3 volume_dims);
};
