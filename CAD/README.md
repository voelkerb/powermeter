[powermeter]: (https://github.com/voelkerb/powermeter)
[bopla]:(https://www.conrad.de/de/p/bopla-eletec-se-432-de-cee-stecker-gehaeuse-120-x-65-x-50-abs-polycarbonat-lichtgrau-graphitgrau-1-st-522228.html)

# Rough PCB Model
You can use the [circuit board](circuitBoard.SLDPRT) design file to design custom enclosures for the PCB.

<img src="/docu/figures/Version2_PCB_Solidworks.png" width="400">

We encourage you to use the off-the-shelf case which can be found at [here](https://www.conrad.de/de/p/bopla-eletec-se-432-de-cee-stecker-gehaeuse-120-x-65-x-50-abs-polycarbonat-lichtgrau-graphitgrau-1-st-522228.html) as you are working with 230V.
The PCB was specially designed for this case and perfectly matches into its mounting holes. Even if the case is bulky and kind of ugly, it offers a secure encapsulation, provides enough room and is rather inexpensive.

As the PCB is designed in Eagle, a 3D part can also be imported directly in Fusion360 (see file [powermeterV2.f3d](powermeterV2.f3d)).

<img src="/docu/figures/Version2_PCB_Fusion.png" width="400">


For a simple stl file which you can also 3D print to see if your casing matches, we refer to [powermeterV2.stl](powermeterV2.stl)).

<img src="/docu/figures/Version2_PCB_STL.png" width="400">

# Casing for a cable
Sometimes the [bopla] case cannot be used as it is a little to bulky. Then you can embed the [powermeter] inside the cable using [case_in_cable_top.STL](case_in_cable_top.STL) and [case_in_cable.STL](case_in_cable.STL). Simply print them and place the PCB inside. 

<img src="/docu/figures/InCableAssembly.png" width="400">
