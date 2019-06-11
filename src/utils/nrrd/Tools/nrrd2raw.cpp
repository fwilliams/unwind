// Created by Andre Aichert on Thu Mar 27th 2014
// Extract raw data chunk from NRRD file
#include <NRRD/nrrd_image.hxx>

template <typename T>
void toraw(const std::string& nrrd_file, const std::string& raw_file)
{
	NRRD::Image<T> img(nrrd_file);
	if (!img)
	{
		std::cerr << "Failed to load image " << nrrd_file << std::endl;
		exit(1);
	}
	std::ofstream raw(raw_file, std::ios::binary);
	if (!raw)
	{
		std::cerr << "Failed to open " << raw_file << " for output!" << std::endl;
		exit(1);
	}
	std::cout << "type:       " << typeName<T>() << std::endl;
	std::cout << "voxels:     " << img.length() << std::endl;
	std::cout << "dimension:  " << img.dimension() << std::endl;
	std::cout << "size:      ";
	for (int i=0;i<img.dimension();i++)
		std::cout << " " << img.size(i);
	std::cout << std::endl;
	raw.write((char *)(T*)img,img.length()*sizeof(T));
}

int main(int argc, char ** argv)
{
	if (argc!=2 && argc!=3)
	{
		std::cerr << "Usage:\n   nrrd2raw file.nrrd [file.raw]\n";
		return 1;
	}

	std::string file_nrrd=argv[1];
	std::string file_raw =argc==3?argv[2]:std::string(argv[1])+".raw";

	std::string type=NRRD::getDataType(file_nrrd);
	// Select correct template based on type
	if (false);
	#define _DEFINE_TYPE_NO_BOOL
	#define _DEFINE_TYPE_NO_LONG
	#define _DEFINE_TYPE(X) else if (type==#X) toraw<X>(file_nrrd,file_raw);
	#include "GetSet/BaseTypes.hxx"
	#undef _DEFINE_TYPE_NO_LONG
	#undef _DEFINE_TYPE_NO_BOOL
	else
	{
		std::cerr << "Failed to load NRRD file " << file_nrrd << std::endl;
		return 1;
	}
	return 0;
}
