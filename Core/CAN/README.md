# CAN
CAN bus related files 
Root directory contains the .dbc files for the CAN matrix.
The C library files are contain within the 'lib' folder. 
Note that this repository has a GitHub Action set up which updates the C library files if a change is pushed to the .dbc files.

Add this reopsitory as a submodule to your project:
```console
$ cd /path/to/your/project/
$ git submodule add git@github.com:kthfspe/CAN.git ./Core/CAN
```
To update the submodule run the following command in your repository folder:
```console
$ git pull --recurse-submodules
```

## Automatically generate C-library

Create a new file in your local repository at .git/hooks/ called pre-commit.
Add this 
```
#!/bin/sh

#export PATH="$HOME/.local/bin:$PATH"

if command -v cantools >/dev/null 2>&1 ; then
	echo "cantools version $(cantools --version) found"
else
	echo "cantools not found. aborting."
	exit
fi

for f in ./*.dbc
do
	echo "Generating $f C library"
	cantools generate_c_source $f
done


mkdir -p lib
mv *.c *.h lib/

git add lib
```

to .git/hooks/pre-commit
