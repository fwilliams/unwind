// Created by Andre Aichert on Mon Sept 2nd 2013

#include "GetSet/StringUtil.hxx"
#include "GetSet/GetSetAutoGui.hxx"
#include "GetSet/GetSetCmdLine.hxx"

GETSET_GLOBAL_DICTIONARY

#include <NRRD/nrrd_image.hxx>

template <typename T>
void import()
{
	auto files=GetSet<std::vector<std::string> >("File/Input").getValue();
	int k=files.size()>999?4:3;
	std::string basename=GetSet<>("File/Output");
	std::string extension=splitRight(basename,".");

	int header=GetSet<int>("Raw/Header (bytes)");
	int nx=GetSet<int>("Raw/Size X");
	int ny=GetSet<int>("Raw/Size Y");
	int nz=GetSet<int>("Raw/Size Z");

	int n=(int)files.size();
	for (int i=0;i<n;i++)
	{
		GetSetAutoGUI::progress("Converting RAW...",i,n-1);
		NRRD::Image<T> img(nx,ny,nz);
		std::ifstream raw(files[i],std::ios::binary);
		if (raw && raw.good())
		{
			raw.seekg(header);
			raw.read((char*)((T*)img),nx*ny*nz*sizeof(T));
		}
		else
			GetSetAutoGUI::error("File Access Error",files[i]);
		img.save(basename+toString(i,k)+"."+extension);
	}

}

int main(int argc, char ** argv)
{
	// Define parameters
	GetSetGui::File("File/Input")
		.setExtensions("Raw 3D volumes (*)")
		.setMultiple(true)
		.setDescription("A 3D raw volume.");
	GetSetGui::File("File/Output")
		.setExtensions("NRRD files (*.nrrd);;All Files (*)")
		.setCreateNew(true)
		.setDescription("Base name for output file. A three digit number will be appended");

	GetSet<int>("Raw/Header (bytes)")=0;
	GetSet<int>("Raw/Size X")=256;
	GetSet<int>("Raw/Size Y")=256;
	GetSet<int>("Raw/Size Z")=256;

	GetSetGui::Enum("Raw/Type")
		.setChoices("unsigned char;char;unsigned short;short;unsigned int;int;float;double")
		.setDescription("float (default) double, char short int, unsigned *")
		.setAttribute("CommandLineFlag","-t;--type")
		.setString("float");


	// Object to handle command line (if needed)
	GetSetIO::CmdLineParser cmd;
	cmd.declare();

	// First of all handle "-h" and "--help" flags
	if (argc==1 || argc==2 && (std::string(argv[1])=="-h" || std::string(argv[1])=="--help" ))
	{
		std::cout <<"Usage:\n"
					"   raw2nrrd --xml\n"
					"   raw2nrrd config.ini\n"
					"\n"
					<< cmd.getSynopsis();
					;
		return 0;
	}

	// AutoGUI interface ("--xml" flag and ini-file loading) and command line interface
	if (!GetSetAutoGUI::handleCommandLine(argc,argv) && !cmd.parse(argc, argv) || (cmd.getUnhandledArgs().size()>1))
		GetSetAutoGUI::error("Command Line", "Unrerecognized command line arguments. Try:   raw2nrrd --help");

	std::string type=GetSet<>("Raw/Type");
	// Select correct template based on type
	#define _DEFINE_TYPE_NO_BOOL
	#define _DEFINE_TYPE_NO_CHAR
	#define _DEFINE_TYPE_NO_LONG
	#define _DEFINE_TYPE(X) if (type==#X) import<X>();
	#include "GetSet/BaseTypes.hxx"
	#undef _DEFINE_TYPE_NO_LONG
	#undef _DEFINE_TYPE_NO_CHAR
	#undef _DEFINE_TYPE_NO_BOOL
	return 0;
}
