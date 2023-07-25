cd
cd /
cd $WORKSPACE/raisimLib && cd build &&\
cmake .. -DCMAKE_INSTALL_PREFIX=$LOCAL_BUILD -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DPYTHON_EXECUTABLE=$(python3 -c "import sys; print(sys.executable)") &&\
make install 
cd
cd /

#========================================================================
# cd $WORKSPACE/ogre && git checkout raisimOgre && cd build &&\
cd $WORKSPACE/ogre/build &&\
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$LOCAL_BUILD -DOGRE_BUILD_COMPONENT_BITES=ON -OGRE_BUILD_COMPONENT_JAVA=OFF -DOGRE_BUILD_DEPENDENCIES=OFF -DOGRE_BUILD_SAMPLES=False &&\
make install 
cd
cd /

#========================================================================
cd $WORKSPACE/raisimOgre && cd build &&\
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$LOCAL_BUILD -DCMAKE_INSTALL_PREFIX=$LOCAL_BUILD -DRAISIM_OGRE_EXAMPLES=ON &&\
make install -j24
cd
cd /

#========================================================================
cd $WORKSPACE/pybind11 && cd build &&\
cmake .. -DCMAKE_INSTALL_PREFIX=$LOCAL_BUILD -DPYBIND11_TEST=OFF &&\
make install -j24
cd
cd /

#========================================================================
cd /raisim_workspace/raisimLib/raisimGymTorch/ && \
python3 setup.py develop --CMAKE_PREFIX_PATH $LOCAL_BUILD



