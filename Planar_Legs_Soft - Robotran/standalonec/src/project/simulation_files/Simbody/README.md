Processing contact by Simbody library
=====================================


 General information 
---------------------------
Here is a template for coupling open-source [Simbody library](https://simtk.org/home/simbody/) for contact detecting  and processing with Robotran functions. 
Simbody provides a set of functions for contact detecting and processing. 

There are two algorithms for contact processing: 

1. Hertz contact for objects, which shape can be well approximated by paraboloid -- spheres, ellipsoids; (Not coupled yet with Robotran)

2. Elastic foundation model for mesh-to-mesh contact – for primitives like boxes, spheres and so on. Meshes can be created by means of Simbody functions. 
Also you can load meshes from WaveFront OBJ files. 

Notice that both methods include surface-to-plane contacts. 

The way of coupling Simbody and Robotran is the following: for each body that can contact with other bodies or ground 
(i.e. a body that is fixed in inertial frame) we create a “shadow” body in Simbody world. Then we
apply contact surfaces to it and define mechanical properties of contact – such as coefficients of friction, damping and stiffness (TODO! a link). 
During simulation Robotran S-sensors provide coordinates and velocities for each "shadow" 
body, and Simbody function computes all contacts and returns values of contact forces and torques brought to the origins of the bodies' coordinate frames. 
These values are used in the “user_ExtForces” function of Robotran.

Installation of Simbody 
-------------------------------

Simbody is an open-source, object-oriented C++ API, and its source codes can be download from [here](https://simtk.org/project/xml/downloads.xml?group_id=47).
Note that you should build 64-bit version of this library to be compatible with the whole project.
Instructions of building Simbody can be found on the Simbody official site: 

* [Mac and Linux](https://simtk.org/docman/view.php/47/1583/HowToBuildSimbodyFromSource_MacLinux.pdf)

* [Windows](https://simtk.org/docman/view.php/47/1582/HowToBuildSimbodyFromSource_Windows.pdf) 
(to make 64-bit version it is necessary to specify a generator “Visual Studio 11 Win64” for the Simbody project in Cmake).
Don't forget to build Release versions of the library (not only Debug)!


Definition of contact properties
-----------------------------------
### To define S- and F-sensors ###

In your model, add S- and F-sensors to the bodies that you want to be in contact with ground or with other contact bodies (e.g. you can use Robotran MBSysPad to add these sensors). 
Sensors should be placed in the origin of body coordinate frame. 

### Fill the contact properties ###

First, define the header file "SimbodyBodiesDefinitions.h":

* Define a number of contact bodies (except the ground that does not move during simulation. If in your project you need a moving ground, then you should add it like others bodies).

E.g. `#define NB_CONTACT_BODIES 4`

* Specify arrays for sensors IDs. S- and F- sensors of different bodies should go in the same order in these arrays. 
So the algorithm considers that first members of these arrays are attached to the first body in Simbody world, second members to the second body and so on.

```
	 #define S_SENSORS_ARRAY {1, 5, 4, 9}
	 #define F_SENSORS_ARRAY {4, 7, 6, 5}
```

* Second, specify coefficients of contact model and meshes in the file "ContactPropertiesDefinitions.cpp".
There you have two functions to fill. The first one 'fill_ground_contact_properties' defines properties of ground (TODO! to add a flag to specify if ground exists in the model),
the second one 'fill_bodies_contact_properties' is used for bodies (you should fill the properties in the order you used for sensors in the previous file). 

You need to fill the following fields:

A. Mechanical parameters of the contact (link to the description of this parameters is in the "Comments" paragraph below):

```
 	BodyContProp->ud = 0.9;   // dynamic   dry friction coefficient
	BodyContProp->us = 1.1;   // static;  it is required ud < us
	BodyContProp->uv = 0;     // viscous (force/velocity)
	BodyContProp->c = 1e3;   // dissipation (1/v)
	BodyContProp->k = 5e3;    // stiffness (pascals)
	BodyContProp->thickness = 0.01;
```
	
B. Type of geometry. 
To run faster, some primitive geometries can be used.
 
	* For ground '0' here means OneHalfSpace  z+ (e.g. flat horizontal ground); 
	* '1' means a mesh defined by a file (the more general case but also the slowest).
	* '2' a box  
	* '3' a sphere

Note : the contact between all geometries might not be available (refer to Simbody documentation). E.g. at the present time box contact (simbody bricks) is only available for half space). 

(TODO! may be here -1 will be to specify if there is no ground; 
for bodies we can specify different primitives such as sphere, box, etc.)

```
	BodyContProp->Geometry = 1; // 0 means OneHalfSpace  z+; 1 means a mesh
```

C. Transformation of geometry - you should use the same numbers as in MBSysPad for vrml meshes. 
These fields are only applicable for mesh ground only (TODO! to add possibility to have inclined or  shifted ground)

```
	BodyContProp->ScaleFactor = 0.5; // scaling factor. Applicable only for mesh
	BodyContProp->Transform[0] = -2.5; // translation in X, (TODO! to change name of the field from transform to translate? )
	BodyContProp->Transform[1] = -2; // ... Y,
	BodyContProp->Transform[2] = -0.05; // and Z directions
	BodyContProp->Rotation[0] = 90; // [degrees] Rotation about X,
	BodyContProp->Rotation[1] = 0; // Y,
	BodyContProp->Rotation[2] = 0; // Z axis.
```
	
D. Geometry details
	* For a mesh specify the mesh file

```
	sprintf(BodyContProp->FileName, "ground_mine.obj");  // file should be in the folder \Standalone\src\project\simbody` (TODO! Where?)
```
	* For a box, specify the half-length

```
	CurBodyContProp->box_dim[0] = 0.075;
	CurBodyContProp->box_dim[1] = 0.075;
	CurBodyContProp->box_dim[2] = 0.075;
```
	* For a sphere, specify the radius

```
	CurBodyContProp->radius = 0.02;
```

Switching on flags in CMake
---------------------------------------

You should switch on following flags in CMake to have Simbody in your project:

* __FLAG_CXX_PROJECT__           : ON: cpp files and cc files (among which the main file) are compiled with a C++ compiler - OFF: no C++ used.
* __FLAG_SIMBODY__               : ON: Simbody used to handle the contacts - OFF: no Simbody feature.
* __FLAG_SIMBODYVIZ__            : ON: only for Windows. Add an OpenGL vizualization of Simbody world - it is useful to check if all meshes are attached to bodies and ground clearly - OFF: no Simbody feature.

and you should switch  off
* __FLAG_GROUND_CONTACT_MODEL__  : ON: Manual ground Contact Model (GCM) used - OFF: no the case (only used for the CoMan model).

(TOCHECK! Only for Windows - what for Linux?)
Also check the version of Simbody libraries in __SIMBODY_LIBRARIES_COM___, __-MATH__ and __-SIMB__ fields. By default it should be RELEASE version:
* C:/Program Files/Simbody/lib/SimTKcommon.lib
* C:/Program Files/Simbody/lib/SimTKmath.lib
* C:/Program Files/Simbody/lib/SimTKsimbody.lib
For __DEBUG__ versions there exists a postfix "_d" at the end of the file-name (like SimTKcommon_d.lib). DEBUG versions are very slow.

Comments
------------------------------

The physical meaninigs of the mechanical contact properties are described
[here](https://simtk.org/api_docs/simbody/api_docs33/Simbody/html/classSimTK_1_1ContactMaterial.html#details).

Exact formulae for normal and tangent forces are described 
[here](https://simtk.org/api_docs/simbody/api_docs33/Simbody/html/classSimTK_1_1ElasticFoundationForce.html#details).

To convert meshes to WaveFront *.obj (or *.vrml) format from Collada *.dae files we use [MeshLab](http://meshlab.sourceforge.net/).
Also it is useful for repairing of meshes: finding holes, removing unreferenced vetrices and duplicated vertices or faces.
(Filters->Cleaning and Repairing). Also Simbody requires that a mesh was a closed manifold - that means that vertices of the mesh's triangles 
were listed in the counter-clockwise order if we look at it from the outside of the surface. 
(In case of this problem look at the 'Select non-manifold vertices' command.)
Also we do not advise to use polygonal mesh for a contact - although Simbody has a function to convert polygonal mesh to a trianular one, 
it didn't work properly on some of our examples.

