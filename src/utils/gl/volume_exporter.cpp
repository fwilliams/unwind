#include "volume_exporter.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

#include <glm/gtc/type_ptr.hpp>
#include <igl/opengl/create_shader_program.h>
#include <utils/nrrd/NRRD/nrrd.hxx>

constexpr const char* SLICE_VERTEX_SHADER = R"(
#version 150
// Create two triangles that are filling the entire screen [-1, 1]
vec2 positions[6] = vec2[](
    vec2(-1.0, -1.0),
    vec2( 1.0, -1.0),
    vec2( 1.0,  1.0),

    vec2(-1.0, -1.0),
    vec2( 1.0,  1.0),
    vec2(-1.0,  1.0)
);


uniform vec3 ll;
uniform vec3 lr;
uniform vec3 ul;
uniform vec3 ur;

out vec3 uv;

void main() {
    vec2 p = positions[gl_VertexID];
    gl_Position = vec4(p, 0.0, 1.0);

    switch (gl_VertexID) {
        case 0:
        case 3:
            uv = ll;
            break;
        case 1:
            uv = lr;
            break;
        case 2:
        case 4:
            uv = ur;
            break;
        case 5:
            uv = ul;
            break;
    }
}
)";

constexpr const char* SLICE_FRAGMENT_SHADER = R"(
#version 150
in vec3 uv;

out vec4 out_color;

uniform sampler3D tex;
uniform sampler1D tf;

void main() {
    out_color = vec4(texture(tex, uv).rrr, 1.0);
}
)";

void VolumeExporter::write_texture_data_to_file(std::string raw_filename, std::string nrrd_filename) {
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Export");
    const size_t num_voxels = size_t(w)*size_t(h)*size_t(d);
    std::vector<std::uint8_t> out_data;
    out_data.resize(num_voxels*4);

    glBindTexture(GL_TEXTURE_3D, render_texture);
    glGetTexImage(GL_TEXTURE_3D, 0, GL_RGBA, GL_UNSIGNED_BYTE, (void*)out_data.data());
    glBindTexture(GL_TEXTURE_3D, 0);
    glFinish();

    std::vector<uint8_t> real_data;
    real_data.resize(num_voxels);
    for (size_t i = 0; i < num_voxels; i++) { real_data[i] = out_data[4*i]; }
    std::ofstream fout;
    fout.open(raw_filename, std::ios::binary);
    fout.write(reinterpret_cast<char*>(real_data.data()), num_voxels*sizeof(uint8_t));
    fout.close();
    NRRD::save3D <uint8_t> (nrrd_filename, real_data.data(), this->w, this->h, this->d);
    glPopDebugGroup();
}

void VolumeExporter::set_export_dims(GLsizei w, GLsizei h, GLsizei d) {
    this->w = w;
    this->h = h;
    this->d = d;
    glBindTexture(GL_TEXTURE_3D, render_texture);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RED, w, h, d, 0, GL_RED, GL_UNSIGNED_BYTE, 0);
    glBindTexture(GL_TEXTURE_3D, 0);
}

void VolumeExporter::destroy() {
    glDeleteProgram(slice.program);
    glDeleteFramebuffers(1, &framebuffer);
    glDeleteTextures(1, &render_texture);
    glDeleteVertexArrays(1, &empty_vao);
    w = 0; h = 0; d = 0;
}

void VolumeExporter::init(GLsizei w, GLsizei h, GLsizei d) {
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Init Slice");
    igl::opengl::create_shader_program(SLICE_VERTEX_SHADER,
                                       SLICE_FRAGMENT_SHADER, {}, slice.program);
    slice.ll_location = glGetUniformLocation(slice.program, "ll");
    slice.lr_location = glGetUniformLocation(slice.program, "lr");
    slice.ul_location = glGetUniformLocation(slice.program, "ul");
    slice.ur_location = glGetUniformLocation(slice.program, "ur");
    slice.texture_location = glGetUniformLocation(slice.program, "tex");
    slice.tf_location = glGetUniformLocation(slice.program, "tf");

    glGenVertexArrays(1, &empty_vao);

    glGenTextures(1, &render_texture);
    glBindTexture(GL_TEXTURE_3D, render_texture);
    GLfloat transparent_color[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    glTexParameterfv(GL_TEXTURE_3D, GL_TEXTURE_BORDER_COLOR, transparent_color);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    set_export_dims(w, h, d);

    glGenFramebuffers(1, &framebuffer);

    glBindTexture(GL_TEXTURE_3D, 0);
    glPopDebugGroup();

}

void VolumeExporter::update(BoundingCage& cage, GLuint volume_texture, glm::ivec3 volume_dims) {
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

    GLint old_viewport[4];
    glGetIntegerv(GL_VIEWPORT, old_viewport);

    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Export Slice");
    glUseProgram(slice.program);
    glBindVertexArray(empty_vao);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_3D, volume_texture);
    glUniform1i(slice.texture_location, 0);

    std::vector<double> kf_depths;
    cage.keyframe_depths(kf_depths);
    double cage_length = kf_depths.back();
    int kf_i = 0;
    for (const BoundingCage::Cell& cell : cage.cells) {
        double start_depth = kf_depths[kf_i];
        double end_depth = kf_depths[kf_i + 1];

        int start_frame = int(d * (start_depth / cage_length));
        int end_frame = int(d * (end_depth / cage_length));

        double start_index = cell.left_keyframe()->index();
        double end_index = cell.right_keyframe()->index();

        for (int i = start_frame; i < end_frame; i++) {
            double lam = double(i-start_frame)/double(end_frame-start_frame);
            double index = (1.0-lam)*start_index + lam*end_index;
            BoundingCage::KeyFrameIterator kf = cage.keyframe_for_index(index);

            Eigen::MatrixXd v3d = kf->bounding_box_vertices_3d();
            glm::vec3 ll(v3d(0, 0), v3d(0, 1), v3d(0, 2));
            glm::vec3 lr(v3d(1, 0), v3d(1, 1), v3d(1, 2));
            glm::vec3 ur(v3d(2, 0), v3d(2, 1), v3d(2, 2));
            glm::vec3 ul(v3d(3, 0), v3d(3, 1), v3d(3, 2));

            ll /= glm::vec3(volume_dims);
            ul /= glm::vec3(volume_dims);
            lr /= glm::vec3(volume_dims);
            ur /= glm::vec3(volume_dims);

            glFramebufferTexture3D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_3D, render_texture, 0, i);
            GLenum draw_buffers[1] = {GL_COLOR_ATTACHMENT0};
            glDrawBuffers(1, draw_buffers);
            if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
                exit(EXIT_FAILURE);
            }

            glClearColor(0.f, 0.f, 0.f, 0.f);
            glViewport(0, 0, w, h);
            glClear(GL_COLOR_BUFFER_BIT);

            glUniform3fv(slice.ll_location, 1, glm::value_ptr(ll));
            glUniform3fv(slice.lr_location, 1, glm::value_ptr(lr));
            glUniform3fv(slice.ul_location, 1, glm::value_ptr(ul));
            glUniform3fv(slice.ur_location, 1, glm::value_ptr(ur));

            glDrawArrays(GL_TRIANGLES, 0, 6);
        }

        kf_i += 1;
    }

    glBindVertexArray(0);
    glUseProgram(0);
    glBindTexture(GL_TEXTURE_3D, 0);
    glPopDebugGroup();

    glViewport(old_viewport[0], old_viewport[1], old_viewport[2], old_viewport[3]);
}
