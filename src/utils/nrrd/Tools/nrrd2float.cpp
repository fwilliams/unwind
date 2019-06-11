// Created by Andre Aichert on Thu Mar 27th 2014
// Convert NRRD file to float
#include <NRRD/nrrd_image.hxx>

int main(int argc, char ** argv)
{
	if (argc!=2 && argc!=3)
	{
		std::cerr << "Usage:\n   nrrd2float file.nrrd [file_float.nrrd]\n";
		return 1;
	}

	std::string file_in =argv[1];
	std::string file_out;
	if (argc==3)
		file_out=argv[2];
	else
	{
		file_out=file_in;
		splitRight(file_out,".");
		file_out=file_out+"_float.nrrd";
	}

	NRRD::Image<float> img(file_in);

	std::cout << "size    = ";
	for (int d=0;d<img.dimension();d++)
		std::cout << " " << img.size(d);
	std::cout << std::endl;
	std::cout << "spacing = ";
	for (int d=0;d<img.dimension();d++)
		std::cout << " " << img.spacing(d);
	std::cout << std::endl;
	
	std::cout << "Processsing...\nPlease do not terminante process.\n";

	if (!img)
	{
		std::cerr << "Failed to load NRRD file " << file_in << std::endl;
		return 1;
	}

	if (!img.save(file_out))
	{
		std::cerr << "Failed to save NRRD file " << file_out << std::endl;
		return 2;
	}

	return 0;
}
