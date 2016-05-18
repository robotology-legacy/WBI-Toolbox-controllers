# Model development guidelines 

To ensure that the controllers commited to this repository can be used easily by anyone, we have some guidelines 
to follow before commiting a new model/controller to the repository. 

* The model support Matlab 2012b or any later version. 
* `slx` models should be avoided. 
* Every model should have an associated `README.md`, describing how to use the model. See the for example [controllers/torqueBalancing/README.md](controllers/torqueBalancing/README.md). 
* The model should not be able to work with a plain installation of Matlab and Simulink, without any additional toolbox. 
