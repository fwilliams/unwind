#include <nrrd_image.hxx>

// A program which converts a NRRD file to float type and sets the first pixel to 1.0 in two different ways.
int main(int argc, char ** argv)
{
    // reading a NRRD header without reading the image data:
    std::string file="file.nrrd";
    std::cout << "File: " << file << std::endl;
    std::cout << "Type: " << NRRD::getDataType(file) << std::endl;

    // Loading an image (Note: automatic conversion to float)
    NRRD::Image<float> img(file);
    if (!img) {
        std::cerr << "Failed to read file.nrrd.\n";
        return 1;
    }

    // Supports N-dimensional images
    std::cout << "Number of dimensions: " << img.dimension() << std::endl;
    std::cout << "Size:";
    for (int i=0;i<img.dimension();i++)
        std::cout << " " << img.size(i);
    std::cout << std::endl;

    // Simple read/write access to image data (1D, 2D and 3D only)
    img.pixel(0,0,0)=1.0;

    // Bi/Tri-linear interpolation
    double pixel=img(0.5,0.5,0.0);

    // Access raw data like it were a pointer to an array
    img[0]=1.0;
    float *raw=img;

    // Saving an image
    if (!img.save("file_float.nrrd")) {
        std::cerr << "Failed to save output file.\n";
        return 1;
    }

    return 0;
}
