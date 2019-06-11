// Created by Andre Aichert on Mon Feb 3rd 2014

#include <GetSet/StringUtil.hxx>
#include <GetSet/GetSetAutoGui.hxx>
#include <GetSet/GetSetCmdLine.hxx>

GETSET_GLOBAL_DICTIONARY

#include <NRRD/nrrd_image.hxx>

inline void swap_directions(int direction, int &x, int &y, int &z)
{
	switch(direction)
	{
	default:
	case 0: // fast X-Y-Z slow
		break;
	case 1: // fast X-Z-Y slow
		std::swap(z,y);
		break;
	case 2: // fast Y-X-Z slow
		std::swap(x,y);
		break;
	case 3: // fast Y-Z-X slow
		std::swap(x,y);
		std::swap(y,z);
		break;
	case 4: // fast Z-X-Y slow
		std::swap(x,z);
		std::swap(x,y);
		break;
	case 5: // fast Z-Y-X slow
		std::swap(x,z);
		break;
	}
}

template <typename Tin, typename Tout>
void resample(const std::vector<std::string>& files)
{
	// Get output name
	std::string basename=GetSet<>("File/Output");
	std::string extension=splitRight(basename,".");
	// Sampling
	int    bx=GetSet<int>("Samplig/Binning/Binsize X");
	int    by=GetSet<int>("Samplig/Binning/Binsize Y");
	int    bz=GetSet<int>("Samplig/Binning/Binsize Z");
	int    mode=GetSet<int>("Samplig/Intensity");
	double bias=GetSet<double>("Samplig/Intensity Bias");
	double scale=GetSet<double>("Samplig/Intensity Scale");
	bool   clamp=GetSet<bool>("Samplig/Clamping/Enable");
	double clamp_min=GetSet<double>("Samplig/Clamping/Min");
	double clamp_max=GetSet<double>("Samplig/Clamping/Max");
	bool   removeNaN=GetSet<bool>("Samplig/Set NaN to zero");
	int    fsk=GetSet<int>("Samplig/Frameskip");
	int    order=GetSet<int>("Samplig/Directions");
	bool   flipx=GetSet<bool>("Sampling/Flip Output X");
	bool   flipy=GetSet<bool>("Sampling/Flip Output Y");
	bool   flipz=GetSet<bool>("Sampling/Flip Output Z");

	// Process files
	GetSetAutoGUI::info("Resampling...",std::string("Conveting ")+typeName<Tin>()+" to "+typeName<Tout>()+" and resampling...");
	int n=(int)files.size();
	for (int i=0;i<n;i++)
	{
		if (fsk>0 && i%fsk!=0) continue;
		GetSetAutoGUI::progress("Resampling...",i,n-1);
		NRRD::Image<Tin> in(files[i]);
		int d=in.dimension();
		if (d<2)
			GetSetAutoGUI::error("File Access Error",files[i]);
		int od=in.size(2)*bz;
		if (d==2) {
			bz=1;
			od=1;
		}

		int ow=in.size(0)/bx,oh=in.size(1)/by;
		swap_directions(order,ow,oh,od);
		// Copy data
		NRRD::Image<Tout> out(ow,oh,od);
		double sumv=0;
		double sumvv=0;
		double minv=in[0];
		double maxv=in[0];
		for (int z=0;z<od;z++)
			for (int y=0;y<out.size(1);y++)
				for (int x=0;x<out.size(0);x++)
				{
					Tout& v(out.pixel(x,y,z));
					v=0;
					// Binning
					for (int ix=0;ix<bx;ix++)
						for (int iy=0;iy<by;iy++)
							for (int iz=0;iz<bz;iz++)
							{
								int ox=x*bx+ix;
								int oy=y*by+iy;
								int oz=z*bz+iz;
								swap_directions(order,ox,oy,oz);
								if (flipx) ox=ow-ox-1;
								if (flipy) oy=oh-oy-1;
								if (flipz) oz=od-oz-1;
								v+=(Tout)((double)in(ox,oy,oz)/(bx*by*bz));
							}
					// Remove NaNs
					if (v<0&&v>0 && removeNaN)
						v=0;
					// Compute Statistics
					if (v<minv) minv=v;
					if (v>maxv) maxv=v;
					sumv+=v;
					sumvv+=v*v;
				}

		// Print Statistics
		int n=out.length();
		double mean=sumv/n;
		double stddev=std::sqrt(sumvv/n-mean*mean);
		std::cout <<"Statistics:\n"
			"   mean: " << mean << "\n"
			"   dev:  " << stddev << "\n"
			"   min:  " << minv << "\n"
			"   max:  " << maxv << "\n";

		// Min/Max Normalization 
		if (mode==0)
		{
			double range=maxv-minv;
			#pragma omp parallel for
			for (int i=0;i<n;i++)
				out[i]=(Tout)(((((double)out[i]-minv)/range)+bias)*scale);
		}
	
		// Mean Var Normalization
		if (mode==1)
			#pragma omp parallel for
			for (int i=0;i<n;i++)
				out[i]=(Tout)(((((double)out[i]-mean)/stddev)+bias)*scale);

		// Simply apply bias and scale
		if (mode==2)
			#pragma omp parallel for
			for (int i=0;i<n;i++)
				out[i]=(Tout)(((double)out[i]+bias)*scale);

		// Clamping
		if (clamp)
			#pragma omp parallel for
			for (int i=0;i<n;i++)
			{
				Tout& v(out[n]);
				if (v<clamp_min) v=(Tout)clamp_min;
				if (v>clamp_max) v=(Tout)clamp_max;
			}

		// Copy meta info and save
		out.meta_info=in.meta_info;
		out.save(basename+toString(i,3)+"."+extension);
	}

}

template <typename Tout>
void select_impl(std::string tin, const std::vector<std::string>& files)
{
	// Select correct template based on type
	#define _DEFINE_TYPE_NO_BOOL
	#define _DEFINE_TYPE_NO_CHAR
	#define _DEFINE_TYPE_NO_LONG
	#define _DEFINE_TYPE(X) if (tin==#X) resample<X,Tout>(files);
	#include "GetSet/BaseTypes.hxx"
	#undef _DEFINE_TYPE_NO_LONG
	#undef _DEFINE_TYPE_NO_CHAR
	#undef _DEFINE_TYPE_NO_BOOL
}

int main(int argc, char ** argv)
{
	// Define parameters
	GetSetGui::File("File/Input")
		.setExtensions("3D or 2D NRRD files (*.nrrd);;All Files (*)")
		.setMultiple(true)
		.setAttribute("CommandLineFlag","-i;--in;--input")
		.setDescription("One or more 3D or 2D NRRD files.");
	GetSetGui::File("File/Output")
		.setExtensions("NRRD files (*.nrrd);;All Files (*)")
		.setCreateNew(true)
		.setAttribute("CommandLineFlag","-o;--out;--output")
		.setDescription("Base name for output file. A three digit number will be appended.");
	GetSetGui::Enum("File/Output Type")
		.setChoices("unsigned char;char;unsigned short;short;unsigned int;int;float;double")
		.setDescription("float (default) double, char short int, unsigned *")
		.setAttribute("CommandLineFlag","-t;--type")
		.setString("float");

	// Sampling Tab
	
	// Re-binning
	GetSet<int>("Samplig/Binning/Binsize X").setAttribute("CommandLineFlag","--bin-x")=1;
	GetSet<int>("Samplig/Binning/Binsize Y").setAttribute("CommandLineFlag","--bin-y")=1;
	GetSet<int>("Samplig/Binning/Binsize Z").setAttribute("CommandLineFlag","--bin-z")=1;
	// Intensity normalization, scale and bias
	GetSetGui::Enum("Samplig/Intensity").setChoices("min-max;mean-dev;none").setAttribute("CommandLineFlag","--normalization")=2;
	GetSet<double>("Samplig/Intensity Bias").setAttribute("CommandLineFlag","--intensity-bias")=0;
	GetSet<double>("Samplig/Intensity Scale").setAttribute("CommandLineFlag","--intensity-scale")=1;
	// Clamping
	GetSet<bool>("Samplig/Clamping/Enable").setAttribute("CommandLineFlag","--clamping")=false;
	GetSet<double>("Samplig/Clamping/Min").setAttribute("CommandLineFlag","--clamp-min")=0;
	GetSet<double>("Samplig/Clamping/Max").setAttribute("CommandLineFlag","--clamp-max")=1;
	// Flipping and swapping
	GetSet<bool>("Sampling/Flip Output X").setAttribute("CommandLineFlag","--flip-x")=false;
	GetSet<bool>("Sampling/Flip Output Y").setAttribute("CommandLineFlag","--flip-y")=false;
	GetSet<bool>("Sampling/Flip Output Z").setAttribute("CommandLineFlag","--flip-z")=false;
	GetSetGui::Enum("Samplig/Directions")
		.setChoices("x-y-z;x-z-y;y-x-z;y-z-x;z-x-y;z-y-x")
		.setAttribute("CommandLineFlag","--directions;--fast")
		.setDescription("Change encoding of spacial directions from fast to slow.")=0;
	// Others
	GetSet<int>("Samplig/Frameskip").setDescription("Consider only every n-th file for multiple input files.").setAttribute("CommandLineFlag","--frame-skip");
	GetSet<bool>("Samplig/Set NaN to zero").setAttribute("CommandLineFlag","--ignore-NaN")=false;

	
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

	// Get types
	auto files=GetSet<std::vector<std::string> >("File/Input").getValue();
	std::string tout=GetSet<>("File/Output Type");
	std::string tin=NRRD::getDataType(files[0]);

	// Select correct template based on type
	#define _DEFINE_TYPE_NO_BOOL
	#define _DEFINE_TYPE_NO_CHAR
	#define _DEFINE_TYPE_NO_LONG
	#define _DEFINE_TYPE(X) if (tout==#X) select_impl<X>(tin,files);
	#include "GetSet/BaseTypes.hxx"
	#undef _DEFINE_TYPE_NO_LONG
	#undef _DEFINE_TYPE_NO_CHAR
	#undef _DEFINE_TYPE_NO_BOOL
	return 0;
}
