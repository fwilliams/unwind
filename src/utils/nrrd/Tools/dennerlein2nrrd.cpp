// Created by Andre Aichert on Fri Oct 21st 2016

#include <NRRD/nrrd_image.hxx>

#include <fstream>

void endian(float& v)
{
	char* pv_hi=(char*)&v;
	char* pv_lo=pv_hi+1;
	std::swap(*pv_hi,*pv_lo);
}

int main(int argc, char ** argv)
{
	if (argc!=2 && argc!=3)
	{
		std::cerr << "Usage:\n   dennerlein2nrrd file.bin [file.nrrd]\n";
		return 1;
	}

	std::string file_bin =argv[1];
	std::string file_nrrd=argc==3?argv[2]:std::string(argv[1])+".nrrd";

	std::ifstream dennerlein(file_bin, std::ifstream::ate | std::ifstream::binary);
	if (!dennerlein)
	{
		std::cerr << "File access error " << file_bin << std::endl;
		return 1;
	}

	// Get file size
	unsigned filesize=(unsigned)dennerlein.tellg();
	dennerlein.seekg(std::ios::beg);
	dennerlein.clear();

	// Get dimensions
	short w=0,h=0,d=0;
	dennerlein.read((char*)&w,sizeof(short));
	dennerlein.read((char*)&h,sizeof(short));
	dennerlein.read((char*)&d,sizeof(short));

	unsigned n_bytes=(unsigned)w*h*d*sizeof(float);
	std::cout << w << " x " << h << " x " << d << " (" << n_bytes << " bytes)" << std::endl;

	
	// Check file size and dimensions
	if (filesize != n_bytes + 3*sizeof(short))
	{
		std::cerr << "Warning: Corrupt Dennerlein file: Filesize is " << filesize << " bytes." << std::endl;
		if (filesize<n_bytes)
			return 2;
	}

	// Load
	NRRD::Image<float> img(w,h,d);
	dennerlein.read((char*)((float*)img),n_bytes);

	// Save
	if (!img.save(file_nrrd))
	{
		std::cerr << "File access error " << file_nrrd << std::endl;
		return 3;
	}

	return 0;
}
