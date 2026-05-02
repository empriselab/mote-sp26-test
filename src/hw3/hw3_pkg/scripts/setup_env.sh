#!/bin/bash

# Upgrade numpy if needed
python3 -c "import numpy.typing" 2>/dev/null || pip3 install --upgrade numpy
# install scipy if needed
python3 -c "import scipy" 2>/dev/null || pip3 install scipy

exit 0