## Updating CEF

Until https://bitbucket.org/chromiumembedded/cef/pull-requests/131 or a replacement has been merged, custom built versions of CEF are required with the PR applied to get decent rendering performance.

It is structured so that the contents of the build zip can be extracted to the correct platform folder, and for some of the larger files to be added to relevent the large_files archive. 

### Building
These steps build on the official steps from https://bitbucket.org/chromiumembedded/cef/wiki/MasterBuildQuickStart.md and https://bitbucket.org/chromiumembedded/cef/wiki/BranchesAndBuilding.md

#### Building Windows
1. Follow steps 1-6 from https://bitbucket.org/chromiumembedded/cef/wiki/MasterBuildQuickStart.md
1. Apply any patches to the CEF source
1. Follow steps 7-8 replacing Debug_GN_x86 with Release_GN_x64
1. Package the resulting binaries with the following
	```
	cd /path/to/chromium/src/cef/tools
	./make_distrib.bat --ninja-build --x64-build --minimal
	```
1. Copy the resulting archive from /path/to/chromium/src/cef/binary_distrib

#### Building Linux
1. Follow steps 1-7 from https://bitbucket.org/chromiumembedded/cef/wiki/MasterBuildQuickStart.md
1. Apply any patches to the CEF source
1. Follow steps 8-10 replacing Debug_GN_x64 with Release_GN_x64
1. Package the resulting binaries with the following
1. Package the resulting binaries with the following
	```
	cd /path/to/chromium/src/cef/tools
	./make_distrib.sh --ninja-build --x64-build --minimal
	```
1. Copy the resulting archive from /path/to/chromium/src/cef/binary_distrib

### Notes
Prebuilt binaries of official versions are available from http://opensource.spotify.com/cefbuilds/index.html where the minimal version should be chosen.
