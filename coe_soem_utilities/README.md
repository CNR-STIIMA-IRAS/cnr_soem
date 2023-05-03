![Alt text](media/logo-eureca-3.png)

# COE SOEM UTILITIES #

The package has been developed within the scope of the project [EURECA](www.cleansky-eureca.eu/), funnded from the [Clean Sky](www.cleansky.eu) Joint Undertaking under the [European Union’s Horizon 2020]](https://ec.europa.eu/programmes/horizon2020/)  research and innovation programme under grant agreement nº 738039

> _The EURECA project framework is dedicated to innovate the assembly of aircraft interiors using advanced human-robot collaborative solutions. A pool of devices/frameworks will be deployed for teaming up with human operators in a human-centred assistive environment. The main benefits are the substantial improvement of ergonomics in workloads, the increase in the usability level of assembly actions, the digitalization of procedures with logs and visuals for error prevention through dedicated devices. Solutions are peculiarly designed for addressing both the working conditions and the management of the cabin-cargo installation process, such as limited maneuvering space, limited weight allowed on the cabin floor, reducing lead time and recurring costs. With this aim, EURECA will bring together research advancements spanning across design, manufacturing and control, robotized hardware, and software for the specific use-cases in the cabin-cargo final assembly._



### CanOpen over Ethercat ###

The CanOpen over Ethercat, [CoE](https://www.can-cia.org/fileadmin/resources/documents/proceedings/2005_rostan.pdf) hereafter, is among the most vesatile and used can in the industrial field, thanks to its intrinsic powerful architecture.  The packes should be considered as an extension with planty of utilities of the FOSS project Simple Open EtherCAT Master [SOEM](https://github.com/ros-industrial/ethercat-soem).

Tha package has been designed in order to control the new empowering collaborative robot deployed in EURECA

![Alt text](media/immagine-progetto-small.jpg)


The package is structured in many ROS packages, and specifically:

Package Name | Description
---:|:---
	 [coe_soem_utilites](https://bitbucket.org/iras-ind/coe_soem_utilities/src/master/) | a set of functionalities that ease the development of SOEM based applciations
	 [coe_core](https://bitbucket.org/iras-ind/coe_core/src/master/) | a set of libraries that implements the most important CanOpen structures, and it provides some utilities to deploy DS402 (motor drivers) applications 
	 [coe_driver](https://bitbucket.org/iras-ind/coe_driver/src/master/) |  a ROS based node that deploys the SOEM  exploting a shared-mameory mechanism
	 [coe_hw_plugins](https://bitbucket.org/iras-ind/coe_hw_plugins/src/master/) | the plugins (based on pluginlib) that allows an easy deployment of client applications 


##coe_soem_utilities Design ##

The package is just a library with a few of methods to make easy and fast the usage of SOEM.


###Dependencies ###

The dependencies are limited, and show below in the table (extracted from package.xml) 

```xml
  <buildtool_depend>catkin</buildtool_depend>
  
  <build_depend>boost</build_depend>
  <build_export_depend>boost</build_export_depend>  
  <exec_depend>boost</exec_depend>

  <build_depend>coe_core</build_depend>
  <build_export_depend>coe_core</build_export_depend>
  <exec_depend>coe_core</exec_depend>
  
  <build_depend>roscpp</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <exec_depend>roscpp</exec_depend>

  <!-- ITIA internal dependencies: to be removed by the end of the   project -->  
   
   
   
```
  
### Contribution guidelines ###

The __coe_seom_utilities__ implements a simple mapping of a generic CoE network in a typed shared memory.
f the client application.

### Who do I talk to? ###

* Nicola Pedrocchi, [nicola.pedrocchi@stiima.cnr.it](mailto:nicola.pedrocchi@stiima.cnr.it) 