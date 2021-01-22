#!/bin/sh

__AUTHOR__="Christoph Steup"

set -e

cd $(dirname $0)
echo 'Generating Doxygen code documentation...'
doxygen Doxyfile > doxygen.log 2>&1

if [ -d "html" ] && [ -f "html/index.html" ]; then
    echo '' >&2
    echo 'Documentation generated in ${PWD}/html' >&2
else
    echo '' >&2
    echo 'Warning: No documentation files have been generated!' >&2
    exit 1
fi
