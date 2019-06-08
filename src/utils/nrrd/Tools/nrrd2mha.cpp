// Created by Andre Aichert on Mon Aug 8th 2016
// Convert NRRD file to a simple ITK/VTK MetaIO MHA file
#include <NRRD/nrrd_metaio.hxx>

template <typename T>
void tomha(const std::string& nrrd_file, const std::string& mha_file)
{
	NRRD::Image<T> img(nrrd_file);
	if (!img)
	{
		std::cerr << "Failed to load image " << nrrd_file << std::endl;
		exit(1);
	}
	NRRD::saveMHA<T>(mha_file,img);
}

int main(int argc, char ** argv)
{
	if (argc!=3)
	{
		std::cerr << "Usage:\n   nrrd2mha file.nrrd file.mha\n";
		return 1;
	}

	std::string type=NRRD::getDataType(argv[1]);
	// Select correct template based on type
	if (false);
	#define _DEFINE_TYPE_NO_BOOL
	#define _DEFINE_TYPE_NO_LONG
	#define _DEFINE_TYPE(X) else if (type==#X) tomha<X>(argv[1],argv[2]);
	#include "GetSet/BaseTypes.hxx"
	#undef _DEFINE_TYPE_NO_LONG
	#undef _DEFINE_TYPE_NO_BOOL
	else
	{
		std::cerr << "Failed to load NRRD file " << argv[1] << std::endl;
		return 1;
	}
	return 0;
}
