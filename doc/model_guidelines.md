# Model development guidelines 

To ensure that the controllers commited to this repository can be easily used by anyone, we have some guidelines 
to follow before commiting a new model/controller to the repository. 

* The model support Matlab 2012b or any later version. 
* `slx` models should be avoided since they are zipfile containters, and their maintenance at the source code level is impossible. `mdl` files, instead, offer some possibility of keeping track of the modifications at the source code level.
* Every model should have an associated `README.md`, describing how to use the model. See the for example [controllers/torqueBalancing/README.md](controllers/torqueBalancing/README.md). 
* The model should be able to work with a plain installation of Matlab and Simulink, without any additional toolbox. 
