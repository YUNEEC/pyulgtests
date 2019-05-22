#!/bin/bash

set -e
pushd ${TRAVIS_BUILD_DIR}
pre-commit install
if ! pre-commit run -a ; then
    ls -lh
    git diff --exit-code
    exit 1
fi
popd
