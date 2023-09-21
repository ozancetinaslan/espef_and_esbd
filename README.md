<h1> ESPEF and ESBD </h1>
This repository contains the Autodesk Maya plugin that I have built for my research papers, ESPEF and ESBD. Please check out those papers for the theoretical background and technical details, because I do not prefer to give those details in here.

Let's jump into how to run the code:

<h2> Prerequisites </h2>
Thankfully, there is not much dependencies in my plugin. Clearly, you have to have any version of Autodesk Maya installed in your computer. I personally use Windows 10 operating system in my machine, but this is not a limitation. You can install Maya on any Linux or MacOs.

In order to run the code, I use Microsoft Visual Studio Community Edition. But if you use another OS, you have to set-up your environment for the Maya plug-in development. There are many sources in web to guide you.

My development environment: Windows 10, Autodesk Maya 2020.4 (education version), Visual Studio 2015. Therefore, I will explain how to run this plug-in according to my development environment. If you have a different set-up, please check out web for further assitance.

<h2> Set-up </h2>
1) Download and install Autodesk Maya for Windows
2) Download the dev-kit from Autodesk website
3) Download and install MS Visual Studio (not vscode!)
4) Follow the instructions in dev-kit (it is straightforward! just a couple of copy-paste operations)
5) You should be done right now. Please test your development environment by creating a test project. An Autodesk Wizard should guide you to create your project.
6) If you fail, please check the instructions in dev-kit again. (Maybe just for the first time, you may open Visual Studio in administrator mode! after no need to this also)

<h2> Building the Code </h2>
1) Please download the project "xpbd2020".
2) Try to build it.
3) If it works, good to go.
4) If it does not work, please download the source files.
5) Create a new Maya Plug-in project, name it "xpbd2020", follow the wizard, except "Image" and "libMocap" tick everything.
6) Copy the source files into the folder of xpbd2020 project.
7) Insert these source files to your project.
8) It should compile and build now (I suggest the release mode, not the debug mode).
9) If it does not work, please contact with me :)

<h2> Setting up Maya </h2>
1) Visual Studio should generate a "xpbd2020.mll" file after the built (not a .exe file)
2) Please copy that .mll file into something like "...\documents\maya\plug-ins" folder.
3) Please download the .py files from the scripts folder of this repo.
4) Copy them into something like "...\documents\maya\scripts" folder.
5) Please download the .ma files from the scenes folder of this repo.
6) Copy them into something like "...\documents\maya\projects\default\scenes" folder.

<h2> Running the Cloth Simulation </h2>
1) Please open Maya
2) Go to File - Open Scene... (up left)
3) Select clothFall18.ma file
4) Open the script editor of Maya (down right)
5) type "import xpbd2020Test1"
6) run it

<h2> Running the Beam Simulation </h2>
1) Please open Maya
2) Go to File - Open Scene... (up left)
3) Select beam.ma file
4) Open the script editor of Maya (down right)
5) type "import xpbd2020Test8"
6) run it

<h2> Simulating Different Constraints </h2>

ESPEF and ESBD propose novel position-based constraint to simulate deformable models. Their constraints and more are already implemented in PHYSSolverCPU.cpp file. 

However, our implementation does not provide a runtime constraint change. Therefore, everytime you have to select which constraint you desire to simulate and re-built the project. Copy the the new .mll file to the "...\documents\maya\plug-ins" folder.

You have to search for the "PHYSSolverCPU::satisfyConstraints" function in PHYSSolverCPU.cpp. The constraints are listed there, as show below:


	//satisfyConstraints
	void PHYSSolverCPU::satisfyConstraints(staticSolverData_t &staticSolverData, dynamicSolverData_t &dynamicSolverData)
	{	
	 	//constraint hierarchy 
		//stretchingConstraint(staticSolverData, dynamicSolverData);
		//centerofMassComputation(staticSolverData, dynamicSolverData);
	
		//greenStrainConstraintTri(staticSolverData, dynamicSolverData);
		//greenStrainConstraintModifiedTri(staticSolverData, dynamicSolverData);
		//greenStrainConstraintTet(staticSolverData, dynamicSolverData);
		//greenStrainConstraintModifiedTet(staticSolverData, dynamicSolverData);
		//areaStrainConstraintTri(staticSolverData, dynamicSolverData);
		//volumeStrainConstraintTet(staticSolverData, dynamicSolverData);
	
		//stVenantKirchhoffConstraintTet(staticSolverData, dynamicSolverData);
		//neoHookeanConstraintTet(staticSolverData, dynamicSolverData);
	
		//hookeSpringConstraint(staticSolverData, dynamicSolverData);
		//stvkSpringConstraint(staticSolverData, dynamicSolverData);
		//morsePotentialConstraint(staticSolverData, dynamicSolverData);
		//exponentialHookeSpringConstraint(staticSolverData, dynamicSolverData);
		//exponentialStvkSpringConstraint(staticSolverData, dynamicSolverData);
		//volumeStrainConstraintTet(staticSolverData, dynamicSolverData);
	
		exponentialGreenStrainConstraintTri(staticSolverData, dynamicSolverData);
		//exponentialGreenStrainConstraintModifiedTri(staticSolverData, dynamicSolverData);
		//exponentialGreenStrainConstraintTet(staticSolverData, dynamicSolverData);
		//exponentialGreenStrainConstraintModifiedTet(staticSolverData, dynamicSolverData);
		//areaStrainConstraintTri(staticSolverData, dynamicSolverData);
		//volumeStrainConstraintTet(staticSolverData, dynamicSolverData);
		
		collisionConstraints(staticSolverData, dynamicSolverData);
		positionConstraints(staticSolverData, dynamicSolverData); 
	}

In this list, you have to select your desired constraint and uncomment it and comment out the existing constraint.

For example, in its current version ESBD cloth simulation constraint (exponentialGreenStrainConstraintTri(staticSolverData, dynamicSolverData);) is available. If you build this, you have to run the clothFall18.ma scene

Another example, if you want to simulate the ESBD beam simulation, you have to uncomment the following constraints:

centerofMassComputation(staticSolverData, dynamicSolverData);

exponentialGreenStrainConstraintModifiedTri(staticSolverData, dynamicSolverData);

volumeStrainConstraintTet(staticSolverData, dynamicSolverData);

Comment out the other constraints. After you have to build the code, copy the new .mll file to "...\documents\maya\plug-ins" folder. And run the beam.ma scene.

<h3> IMPORTANT NOTE 1: </h3> 
Please always keep the collisionConstraints(staticSolverData, dynamicSolverData); and positionConstraints(staticSolverData, dynamicSolverData); constraints available.

<h3> IMPORTANT NOTE 2: </h3> 
If you want to simulate the volumetric models, always keep centerofMassComputation(staticSolverData, dynamicSolverData); and volumeStrainConstraintTet(staticSolverData, dynamicSolverData); functions available (except StVK and Neo-hookean constraints which already provide volume!).

<h3> ESBD Constraints are: </h3>

exponentialGreenStrainConstraintTri(staticSolverData, dynamicSolverData);

exponentialGreenStrainConstraintModifiedTri(staticSolverData, dynamicSolverData);

exponentialGreenStrainConstraintTet(staticSolverData, dynamicSolverData);

exponentialGreenStrainConstraintModifiedTet(staticSolverData, dynamicSolverData);

<h3> ESPEF Constraints are: </h3>

morsePotentialConstraint(staticSolverData, dynamicSolverData);

exponentialHookeSpringConstraint(staticSolverData, dynamicSolverData);

exponentialStvkSpringConstraint(staticSolverData, dynamicSolverData);

<h3> Other Constraints are: </h3>

Other state of the art methods for comparison purposes in the publications.

&nbsp;&nbsp;

Please enjoy the project and if you need any assistance, contact with me anytime.

Thank you!
