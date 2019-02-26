#include "state.h"


void State::SegmentedFeatures::recompute_feature_map() {
    selected_features.clear();
    buffer_data.clear();
    features = topological_features.getFeatures(num_selected_features, 0.f);

    uint32_t size = topological_features.ctdata.noArcs;
    buffer_data.resize(size + 1 + 1, static_cast<uint32_t>(-1));
    buffer_data[0] = static_cast<uint32_t>(features.size());
    for (size_t i = 0; i < features.size(); ++i) {
        for (uint32_t j : features[i].arcs) {
            // +1 since the first value of the vector contains the number of features
            buffer_data[j + 1] = static_cast<uint32_t>(i);
        }
    }
}

void State::LoadedVolume::preprocess_volume_texture(std::vector<uint8_t>& byte_data) {
    // Pre-load and normalize data for the low res volume GL texture
    const Eigen::RowVector3i volume_dims = dims();
    const int size = volume_dims[0]*volume_dims[1]*volume_dims[2];
    byte_data.clear();
    byte_data.resize(size);
    const double min_value = min_value;
    const double value_range = max_value - min_value;
    const float* texture_data = volume_data.data();
    std::transform(
        texture_data,
        texture_data + size,
        byte_data.begin(),
        [min_value, value_range](double d) {
            return static_cast<uint8_t>(((d - min_value)/value_range) * std::numeric_limits<uint8_t>::max());
        }
    );
}


void State::load_volume_data(State::LoadedVolume& volume, std::string prefix, bool load_topology) {
    std::string prefix_with_path = input_metadata.output_dir + "/" + prefix;

    // Load the volume data
    volume.metadata = DatFile(prefix_with_path + ".dat", logger);
    load_rawfile(prefix_with_path + ".raw", volume.dims(),
                 volume.volume_data, logger, true /* normalize */);
    volume.max_value = volume.volume_data.maxCoeff();
    volume.min_value = volume.volume_data.minCoeff();

    if (load_topology) {
        // Compute the topological features
        Eigen::Vector3i lrv = volume.dims();
        preProcessing(prefix_with_path, lrv[0], lrv[1], lrv[2]);
        segmented_features.topological_features.loadData(prefix_with_path);
        segmented_features.recompute_feature_map();

        // Load the low-res index data
        const size_t num_bytes = volume.num_voxels() * sizeof(uint32_t);
        std::ifstream file;
        file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        file.open(prefix_with_path + ".part.raw", std::ifstream::binary);
        typedef decltype(volume.index_data) IndexType;
        volume.index_data.resize(num_bytes / sizeof(IndexType::Scalar));
        file.read(reinterpret_cast<char*>(volume.index_data.data()), num_bytes);
    }
}


void State::LoadedVolume::load_gl_volume_texture(const std::vector<uint8_t>& byte_data) {
    if (byte_data.size() == 0) {
        return;
    }
    if (volume_texture != 0) {
        glDeleteTextures(1, &volume_texture);
    }

    const Eigen::RowVector3i volume_dims = dims();

    glGenTextures(1, &volume_texture);
    glBindTexture(GL_TEXTURE_3D, volume_texture);
    GLfloat transparent_color[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    glTexParameterfv(GL_TEXTURE_3D, GL_TEXTURE_BORDER_COLOR, transparent_color);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
    //    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    //    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    //    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    const uint8_t* volume_data = byte_data.data();
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RED, volume_dims[0], volume_dims[1], volume_dims[2], 0,
                 GL_RED, GL_UNSIGNED_BYTE, volume_data);
}

void State::LoadedVolume::load_gl_index_texture() {
    if (index_data.size() == 0) {
        return;
    }
    if (index_texture != 0) {
        glDeleteTextures(1, &index_texture);
    }

    const Eigen::RowVector3i volume_dims = dims();

    glBindTexture(GL_TEXTURE_3D, index_texture);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    uint32_t* idata = index_data.data();
    glTexImage3D(GL_TEXTURE_3D, 0, GL_R32UI, volume_dims[0], volume_dims[1], volume_dims[2],
                 0, GL_RED_INTEGER, GL_UNSIGNED_INT, idata);
}

void State::serialize(std::vector<char> &buffer) const {
    igl::serialize(input_metadata.input_dir, std::string("image_input.input_dir"), buffer);
    igl::serialize(input_metadata.output_dir, std::string("image_input.output_dir"), buffer);
    igl::serialize(input_metadata.file_extension, std::string("image_input.file_extension"), buffer);
    igl::serialize(input_metadata.prefix, std::string("image_input.prefix"), buffer);
    igl::serialize(input_metadata.downsample_factor, std::string("image_input.downsample_factor"), buffer);
    igl::serialize(input_metadata.start_index, std::string("image_input.start_index"), buffer);
    igl::serialize(input_metadata.end_index, std::string("image_input.end_index"), buffer);
    igl::serialize(input_metadata.project_name, std::string("image_input.project_name"), buffer);


    igl::serialize(dilated_tet_mesh.TV, std::string("dilated_tet_mesh.TV"), buffer);
    igl::serialize(dilated_tet_mesh.TT, std::string("dilated_tet_mesh.TT"), buffer);
    igl::serialize(dilated_tet_mesh.TF, std::string("dilated_tet_mesh.TF"), buffer);
    igl::serialize(dilated_tet_mesh.connected_components, std::string("dilated_tet_mesh.connected_components"), buffer);
    igl::serialize(dilated_tet_mesh.dilation_radius, std::string("dilated_tet_mesh.dilation_radius"), buffer);
    igl::serialize(dilated_tet_mesh.meshing_voxel_radius, std::string("dilated_tet_mesh.meshing_voxel_radius"), buffer);
    igl::serialize(dilated_tet_mesh.geodesic_dists, std::string("dilated_tet_mesh.geodesic_dists"), buffer);


    igl::serialize(skeleton_estimation_parameters.num_subdivisions, std::string("skeleton_estimation_parameters.num_subdivisions"), buffer);
    igl::serialize(skeleton_estimation_parameters.num_smoothing_iters, std::string("skeleton_estimation_parameters.num_smoothing_iters"), buffer);
    igl::serialize(skeleton_estimation_parameters.cage_bbox_radius, std::string("skeleton_estimation_parameters.cage_bbox_radius"), buffer);
    igl::serialize(skeleton_estimation_parameters.endpoint_pairs, std::string("skeleton_estimation_parameters.endpoint_pairs"), buffer);


    igl::serialize(segmented_features.selected_features, std::string("segmented_features.selected_features"), buffer);
    igl::serialize(segmented_features.num_selected_features, std::string("segmented_features.num_selected_features"), buffer);

    igl::serialize(dirty_flags.file_loading_dirty, std::string("dirty_flags.file_loading_dirty"), buffer);
    igl::serialize(dirty_flags.mesh_dirty, std::string("dirty_flags.mesh_dirty"), buffer);
    igl::serialize(dirty_flags.endpoints_dirty, std::string("dirty_flags.endpoints_dirty"), buffer);
    igl::serialize(dirty_flags.bounding_cage_dirty, std::string("dirty_flags.bounding_cage_dirty"), buffer);

    igl::serialize(cage, std::string("cage"), buffer);
}

void State::deserialize(const std::vector<char> &buffer) {
    igl::deserialize(input_metadata.input_dir, std::string("image_input.input_dir"), buffer);
    igl::deserialize(input_metadata.output_dir, std::string("image_input.output_dir"), buffer);
    igl::deserialize(input_metadata.file_extension, std::string("image_input.file_extension"), buffer);
    igl::deserialize(input_metadata.prefix, std::string("image_input.prefix"), buffer);
    igl::deserialize(input_metadata.downsample_factor, std::string("image_input.downsample_factor"), buffer);
    igl::deserialize(input_metadata.start_index, std::string("image_input.start_index"), buffer);
    igl::deserialize(input_metadata.end_index, std::string("image_input.end_index"), buffer);
    igl::deserialize(input_metadata.project_name, std::string("image_input.project_name"), buffer);


    igl::deserialize(dilated_tet_mesh.TV, std::string("dilated_tet_mesh.TV"), buffer);
    igl::deserialize(dilated_tet_mesh.TT, std::string("dilated_tet_mesh.TT"), buffer);
    igl::deserialize(dilated_tet_mesh.TF, std::string("dilated_tet_mesh.TF"), buffer);
    igl::deserialize(dilated_tet_mesh.connected_components, std::string("dilated_tet_mesh.connected_components"), buffer);
    igl::deserialize(dilated_tet_mesh.dilation_radius, std::string("dilated_tet_mesh.dilation_radius"), buffer);
    igl::deserialize(dilated_tet_mesh.meshing_voxel_radius, std::string("dilated_tet_mesh.meshing_voxel_radius"), buffer);
    igl::deserialize(dilated_tet_mesh.geodesic_dists, std::string("dilated_tet_mesh.geodesic_dists"), buffer);


    igl::deserialize(skeleton_estimation_parameters.num_subdivisions, std::string("skeleton_estimation_parameters.num_subdivisions"), buffer);
    igl::deserialize(skeleton_estimation_parameters.num_smoothing_iters, std::string("skeleton_estimation_parameters.num_smoothing_iters"), buffer);
    igl::deserialize(skeleton_estimation_parameters.cage_bbox_radius, std::string("skeleton_estimation_parameters.cage_bbox_radius"), buffer);
    igl::deserialize(skeleton_estimation_parameters.endpoint_pairs, std::string("skeleton_estimation_parameters.endpoint_pairs"), buffer);


    igl::deserialize(segmented_features.num_selected_features, std::string("segmented_features.num_selected_features"), buffer);
    igl::deserialize(segmented_features.selected_features, std::string("segmented_features.selected_features"), buffer);

    igl::deserialize(dirty_flags.file_loading_dirty, std::string("dirty_flags.file_loading_dirty"), buffer);
    igl::deserialize(dirty_flags.mesh_dirty, std::string("dirty_flags.mesh_dirty"), buffer);
    igl::deserialize(dirty_flags.endpoints_dirty, std::string("dirty_flags.endpoints_dirty"), buffer);
    igl::deserialize(dirty_flags.bounding_cage_dirty, std::string("dirty_flags.bounding_cage_dirty"), buffer);

    igl::deserialize(cage, std::string("cage"), buffer);


    // NOTE: You still need to load the GL textures after serializing by calling
    // state.low_res_texture.load_gl_*()
    // and the same for the hi-res texture
}
