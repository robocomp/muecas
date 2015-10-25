```
```
#
``` phidgetIMU
```
Intro to component here

Go to this page: http://www.phidgets.com/old_drivers.php and download the latest driver to your favourite software directory.
Follow instructions in README file:
	untar
	./configure
	make
	sudo make install


## Configuration parameters
As any other component,
``` *phidgetIMU* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <phidgetIMU 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```phidgetIMU ```

    --Ice.Config=config
