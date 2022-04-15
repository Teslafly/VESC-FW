# Setting up vscode fore vesc development:

Install these extensions:
* Required:
    * https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools
    * https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug
    * https://marketplace.visualstudio.com/items?itemName=ms-vscode.makefile-tools
* Recommended:
    * https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack
    * https://marketplace.visualstudio.com/items?itemName=ZixuanWang.linkerscript
    * https://marketplace.visualstudio.com/items?itemName=letmaik.git-tree-compare
    * https://marketplace.visualstudio.com/items?itemName=mhutchie.git-graph

Follow vesc readme setup instructions.
You will also need openocd installed on your system and in your path.
* windows, get openocd here: https://gnutoolchains.com/arm-eabi/openocd/
    * you must manually add it to your path.
* linux, openocd is probably installable from a package nanager.

In the "makefile" extension you should now be able to set your Build target and Launch tartet. Configuration is unset.

You should now be able to build and debug (F5)





