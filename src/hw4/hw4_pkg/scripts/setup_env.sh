#!/bin/bash

# Upgrade matplotlib if needed
python3 -c "import matplotlib" 2>/dev/null || pip3 install --upgrade matplotlib

exit 0