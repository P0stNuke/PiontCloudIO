QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    colorselection.cpp \
    dialog_alignppp.cpp \
    dialog_icp.cpp \
    dialog_icptv.cpp \
    filter_voxel.cpp \
    form_ppp.cpp \
    main.cpp \
    mainwindow.cpp \
    pcl_fuction.cpp \
    view_pointsizesetting.cpp \
    view_rendering.cpp

HEADERS += \
    colorselection.h \
    dialog_alignppp.h \
    dialog_icp.h \
    dialog_icptv.h \
    filter_voxel.h \
    form_ppp.h \
    mainwindow.h \
    pcl_fuction.h \
    view_pointsizesetting.h \
    view_rendering.h

FORMS += \
    dialog_alignppp.ui \
    dialog_icp.ui \
    dialog_icptv.ui \
    filter_voxel.ui \
    form_ppp.ui \
    mainwindow.ui \
    view_pointsizesetting.ui \
    view_rendering.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

PCL_ROOT        =   $$quote(C:/Tools/PCL 1.13.0)
3rdParty_Qhull  =   $$quote(C:/Tools/PCL 1.13.0/3rdParty/Qhull)
3rdParty_FLANN  =   $$quote(C:/Tools/PCL 1.13.0/3rdParty/FLANN)
3rdParty_Boost  =   $$quote(C:/Tools/PCL 1.13.0/3rdParty/Boost)
3rdParty_Eigen  =   $$quote(C:/Tools/PCL 1.13.0/3rdParty/Eigen)
3rdParty_VTK    =   $$quote(C:/Tools/PCL 1.13.0/3rdParty/VTK)
OpenNI_ROOT     =   $$quote(C:/Tools/PCL 1.13.0/3rdParty/OpenNI2)

INCLUDEPATH += \
    $$PCL_ROOT/include/pcl-1.13 \
    $$3rdParty_Qhull/include \
    $$3rdParty_FLANN/include \
    $$3rdParty_Boost/include/boost-1_80 \
    $$3rdParty_Eigen/eigen3 \
    $$3rdParty_VTK/include/vtk-9.2\
    $$OpenNI_ROOT/Include

LIBS+= \
    -L$$PCL_ROOT/lib \
    -L$$3rdParty_Qhull/lib \
    -L$$3rdParty_FLANN/lib \
    -L$$3rdParty_Boost/lib \
    -L$$3rdParty_Eigen/lib \
    -L$$3rdParty_VTK/lib \
    -L$$OpenNI_ROOT/Lib

LIBS += -lOpenNI2

win32:CONFIG(release, debug|release): LIBS += \
    -lpcl_common \
    -lpcl_features \
    -lpcl_filters \
    -lpcl_io \
    -lpcl_io_ply \
    -lpcl_kdtree \
    -lpcl_keypoints \
    -lpcl_ml \
    -lpcl_octree \
    -lpcl_outofcore \
    -lpcl_people \
    -lpcl_recognition \
    -lpcl_registration \
    -lpcl_sample_consensus \
    -lpcl_search \
    -lpcl_segmentation \
    -lpcl_stereo \
    -lpcl_surface \
    -lpcl_tracking \
    -lpcl_visualization \
    -lvtkcgns-9.2 \
    -lvtkChartsCore-9.2 \
    -lvtkCommonColor-9.2 \
    -lvtkCommonComputationalGeometry-9.2 \
    -lvtkCommonCore-9.2 \
    -lvtkCommonDataModel-9.2 \
    -lvtkCommonExecutionModel-9.2 \
    -lvtkCommonMath-9.2 \
    -lvtkCommonMisc-9.2 \
    -lvtkCommonSystem-9.2 \
    -lvtkCommonTransforms-9.2 \
    -lvtkDICOMParser-9.2 \
    -lvtkDomainsChemistry-9.2 \
    -lvtkDomainsChemistryOpenGL2-9.2 \
    -lvtkdoubleconversion-9.2 \
    -lvtkexodusII-9.2 \
    -lvtkexpat-9.2 \
    -lvtkFiltersAMR-9.2 \
    -lvtkFiltersCore-9.2 \
    -lvtkFiltersExtraction-9.2 \
    -lvtkFiltersFlowPaths-9.2 \
    -lvtkFiltersGeneral-9.2 \
    -lvtkFiltersGeneric-9.2 \
    -lvtkFiltersGeometry-9.2 \
    -lvtkFiltersHybrid-9.2 \
    -lvtkFiltersHyperTree-9.2 \
    -lvtkFiltersImaging-9.2 \
    -lvtkFiltersModeling-9.2 \
    -lvtkFiltersParallel-9.2 \
    -lvtkFiltersParallelImaging-9.2 \
    -lvtkFiltersPoints-9.2 \
    -lvtkFiltersProgrammable-9.2 \
    -lvtkFiltersSelection-9.2 \
    -lvtkFiltersSMP-9.2 \
    -lvtkFiltersSources-9.2 \
    -lvtkFiltersStatistics-9.2 \
    -lvtkFiltersTexture-9.2 \
    -lvtkFiltersTopology-9.2 \
    -lvtkFiltersVerdict-9.2 \
    -lvtkfmt-9.2 \
    -lvtkfreetype-9.2 \
    -lvtkGeovisCore-9.2 \
    -lvtkgl2ps-9.2 \
    -lvtkglew-9.2 \
    -lvtkhdf5-9.2 \
    -lvtkhdf5_hl-9.2 \
    -lvtkImagingColor-9.2 \
    -lvtkImagingCore-9.2 \
    -lvtkImagingFourier-9.2 \
    -lvtkImagingGeneral-9.2 \
    -lvtkImagingHybrid-9.2 \
    -lvtkImagingMath-9.2 \
    -lvtkImagingMorphological-9.2 \
    -lvtkImagingSources-9.2 \
    -lvtkImagingStatistics-9.2 \
    -lvtkImagingStencil-9.2 \
    -lvtkInfovisCore-9.2 \
    -lvtkInfovisLayout-9.2 \
    -lvtkInteractionImage-9.2 \
    -lvtkInteractionStyle-9.2 \
    -lvtkInteractionWidgets-9.2 \
    -lvtkIOAMR-9.2 \
    -lvtkIOAsynchronous-9.2 \
    -lvtkIOCesium3DTiles-9.2 \
    -lvtkIOCGNSReader-9.2 \
    -lvtkIOChemistry-9.2 \
    -lvtkIOCityGML-9.2 \
    -lvtkIOCONVERGECFD-9.2 \
    -lvtkIOCore-9.2 \
    -lvtkIOEnSight-9.2 \
    -lvtkIOExodus-9.2 \
    -lvtkIOExport-9.2 \
    -lvtkIOExportGL2PS-9.2 \
    -lvtkIOExportPDF-9.2 \
    -lvtkIOGeometry-9.2 \
    -lvtkIOHDF-9.2 \
    -lvtkIOImage-9.2 \
    -lvtkIOImport-9.2 \
    -lvtkIOInfovis-9.2 \
    -lvtkIOIOSS-9.2 \
    -lvtkIOLegacy-9.2 \
    -lvtkIOLSDyna-9.2 \
    -lvtkIOMINC-9.2 \
    -lvtkIOMotionFX-9.2 \
    -lvtkIOMovie-9.2 \
    -lvtkIONetCDF-9.2 \
    -lvtkIOOggTheora-9.2 \
    -lvtkIOParallel-9.2 \
    -lvtkIOParallelXML-9.2 \
    -lvtkIOPLY-9.2 \
    -lvtkIOSegY-9.2 \
    -lvtkIOSQL-9.2 \
    -lvtkioss-9.2 \
    -lvtkIOTecplotTable-9.2 \
    -lvtkIOVeraOut-9.2 \
    -lvtkIOVideo-9.2 \
    -lvtkIOXML-9.2 \
    -lvtkIOXMLParser-9.2 \
    -lvtkjpeg-9.2 \
    -lvtkjsoncpp-9.2 \
    -lvtkkissfft-9.2 \
    -lvtklibharu-9.2 \
    -lvtklibproj-9.2 \
    -lvtklibxml2-9.2 \
    -lvtkloguru-9.2 \
    -lvtklz4-9.2 \
    -lvtklzma-9.2 \
    -lvtkmetaio-9.2 \
    -lvtknetcdf-9.2 \
    -lvtkogg-9.2 \
    -lvtkParallelCore-9.2 \
    -lvtkParallelDIY-9.2 \
    -lvtkpng-9.2 \
    -lvtkpugixml-9.2 \
    -lvtkRenderingAnnotation-9.2 \
    -lvtkRenderingContext2D-9.2 \
    -lvtkRenderingContextOpenGL2-9.2 \
    -lvtkRenderingCore-9.2 \
    -lvtkRenderingFreeType-9.2 \
    -lvtkRenderingGL2PSOpenGL2-9.2 \
    -lvtkRenderingHyperTreeGrid-9.2 \
    -lvtkRenderingImage-9.2 \
    -lvtkRenderingLabel-9.2 \
    -lvtkRenderingLICOpenGL2-9.2 \
    -lvtkRenderingLOD-9.2 \
    -lvtkRenderingOpenGL2-9.2 \
    -lvtkRenderingSceneGraph-9.2 \
    -lvtkRenderingUI-9.2 \
    -lvtkRenderingVolume-9.2 \
    -lvtkRenderingVolumeOpenGL2-9.2 \
    -lvtkRenderingVtkJS-9.2 \
    -lvtksqlite-9.2 \
    -lvtksys-9.2 \
    -lvtkTestingRendering-9.2 \
    -lvtktheora-9.2 \
    -lvtktiff-9.2 \
    -lvtkverdict-9.2 \
    -lvtkViewsContext2D-9.2 \
    -lvtkViewsCore-9.2 \
    -lvtkViewsInfovis-9.2 \
    -lvtkWrappingTools-9.2 \
    -lvtkzlib-9.2 \
    -lvtkGUISupportQt-9.2 \
    -lvtkGUISupportQtQuick-9.2 \
    -lvtkGUISupportQtSQL-9.2 \
    -lvtkRenderingQt-9.2 \
    -lvtkViewsQt-9.2


else:win32:CONFIG(debug, debug|release): LIBS += \
    -lpcl_commond \
    -lpcl_featuresd \
    -lpcl_filtersd \
    -lpcl_iod \
    -lpcl_io_plyd \
    -lpcl_kdtreed \
    -lpcl_keypointsd \
    -lpcl_mld \
    -lpcl_octreed \
    -lpcl_outofcored \
    -lpcl_peopled \
    -lpcl_recognitiond \
    -lpcl_registrationd \
    -lpcl_sample_consensusd \
    -lpcl_searchd \
    -lpcl_segmentationd \
    -lpcl_stereod \
    -lpcl_surfaced \
    -lpcl_trackingd \
    -lpcl_visualizationd \
    -lvtkcgns-9.2-gd \
    -lvtkChartsCore-9.2-gd \
    -lvtkCommonColor-9.2-gd \
    -lvtkCommonComputationalGeometry-9.2-gd \
    -lvtkCommonCore-9.2-gd \
    -lvtkCommonDataModel-9.2-gd \
    -lvtkCommonExecutionModel-9.2-gd \
    -lvtkCommonMath-9.2-gd \
    -lvtkCommonMisc-9.2-gd \
    -lvtkCommonSystem-9.2-gd \
    -lvtkCommonTransforms-9.2-gd \
    -lvtkDICOMParser-9.2-gd \
    -lvtkDomainsChemistry-9.2-gd \
    -lvtkDomainsChemistryOpenGL2-9.2-gd \
    -lvtkdoubleconversion-9.2-gd \
    -lvtkexodusII-9.2-gd \
    -lvtkexpat-9.2-gd \
    -lvtkFiltersAMR-9.2-gd \
    -lvtkFiltersCore-9.2-gd \
    -lvtkFiltersExtraction-9.2-gd \
    -lvtkFiltersFlowPaths-9.2-gd \
    -lvtkFiltersGeneral-9.2-gd \
    -lvtkFiltersGeneric-9.2-gd \
    -lvtkFiltersGeometry-9.2-gd \
    -lvtkFiltersHybrid-9.2-gd \
    -lvtkFiltersHyperTree-9.2-gd \
    -lvtkFiltersImaging-9.2-gd \
    -lvtkFiltersModeling-9.2-gd \
    -lvtkFiltersParallel-9.2-gd \
    -lvtkFiltersParallelImaging-9.2-gd \
    -lvtkFiltersPoints-9.2-gd \
    -lvtkFiltersProgrammable-9.2-gd \
    -lvtkFiltersSelection-9.2-gd \
    -lvtkFiltersSMP-9.2-gd \
    -lvtkFiltersSources-9.2-gd \
    -lvtkFiltersStatistics-9.2-gd \
    -lvtkFiltersTexture-9.2-gd \
    -lvtkFiltersTopology-9.2-gd \
    -lvtkFiltersVerdict-9.2-gd \
    -lvtkfmt-9.2-gd \
    -lvtkfreetype-9.2-gd \
    -lvtkGeovisCore-9.2-gd \
    -lvtkgl2ps-9.2-gd \
    -lvtkglew-9.2-gd \
    -lvtkhdf5-9.2-gd \
    -lvtkhdf5_hl-9.2-gd \
    -lvtkImagingColor-9.2-gd \
    -lvtkImagingCore-9.2-gd \
    -lvtkImagingFourier-9.2-gd \
    -lvtkImagingGeneral-9.2-gd \
    -lvtkImagingHybrid-9.2-gd \
    -lvtkImagingMath-9.2-gd \
    -lvtkImagingMorphological-9.2-gd \
    -lvtkImagingSources-9.2-gd \
    -lvtkImagingStatistics-9.2-gd \
    -lvtkImagingStencil-9.2-gd \
    -lvtkInfovisCore-9.2-gd \
    -lvtkInfovisLayout-9.2-gd \
    -lvtkInteractionImage-9.2-gd \
    -lvtkInteractionStyle-9.2-gd \
    -lvtkInteractionWidgets-9.2-gd \
    -lvtkIOAMR-9.2-gd \
    -lvtkIOAsynchronous-9.2-gd \
    -lvtkIOCesium3DTiles-9.2-gd \
    -lvtkIOCGNSReader-9.2-gd \
    -lvtkIOChemistry-9.2-gd \
    -lvtkIOCityGML-9.2-gd \
    -lvtkIOCONVERGECFD-9.2-gd \
    -lvtkIOCore-9.2-gd \
    -lvtkIOEnSight-9.2-gd \
    -lvtkIOExodus-9.2-gd \
    -lvtkIOExport-9.2-gd \
    -lvtkIOExportGL2PS-9.2-gd \
    -lvtkIOExportPDF-9.2-gd \
    -lvtkIOGeometry-9.2-gd \
    -lvtkIOHDF-9.2-gd \
    -lvtkIOImage-9.2-gd \
    -lvtkIOImport-9.2-gd \
    -lvtkIOInfovis-9.2-gd \
    -lvtkIOIOSS-9.2-gd \
    -lvtkIOLegacy-9.2-gd \
    -lvtkIOLSDyna-9.2-gd \
    -lvtkIOMINC-9.2-gd \
    -lvtkIOMotionFX-9.2-gd \
    -lvtkIOMovie-9.2-gd \
    -lvtkIONetCDF-9.2-gd \
    -lvtkIOOggTheora-9.2-gd \
    -lvtkIOParallel-9.2-gd \
    -lvtkIOParallelXML-9.2-gd \
    -lvtkIOPLY-9.2-gd \
    -lvtkIOSegY-9.2-gd \
    -lvtkIOSQL-9.2-gd \
    -lvtkioss-9.2-gd \
    -lvtkIOTecplotTable-9.2-gd \
    -lvtkIOVeraOut-9.2-gd \
    -lvtkIOVideo-9.2-gd \
    -lvtkIOXML-9.2-gd \
    -lvtkIOXMLParser-9.2-gd \
    -lvtkjpeg-9.2-gd \
    -lvtkjsoncpp-9.2-gd \
    -lvtkkissfft-9.2-gd \
    -lvtklibharu-9.2-gd \
    -lvtklibproj-9.2-gd \
    -lvtklibxml2-9.2-gd \
    -lvtkloguru-9.2-gd \
    -lvtklz4-9.2-gd \
    -lvtklzma-9.2-gd \
    -lvtkmetaio-9.2-gd \
    -lvtknetcdf-9.2-gd \
    -lvtkogg-9.2-gd \
    -lvtkParallelCore-9.2-gd \
    -lvtkParallelDIY-9.2-gd \
    -lvtkpng-9.2-gd \
    -lvtkpugixml-9.2-gd \
    -lvtkRenderingAnnotation-9.2-gd \
    -lvtkRenderingContext2D-9.2-gd \
    -lvtkRenderingContextOpenGL2-9.2-gd \
    -lvtkRenderingCore-9.2-gd \
    -lvtkRenderingFreeType-9.2-gd \
    -lvtkRenderingGL2PSOpenGL2-9.2-gd \
    -lvtkRenderingHyperTreeGrid-9.2-gd \
    -lvtkRenderingImage-9.2-gd \
    -lvtkRenderingLabel-9.2-gd \
    -lvtkRenderingLICOpenGL2-9.2-gd \
    -lvtkRenderingLOD-9.2-gd \
    -lvtkRenderingOpenGL2-9.2-gd \
    -lvtkRenderingSceneGraph-9.2-gd \
    -lvtkRenderingUI-9.2-gd \
    -lvtkRenderingVolume-9.2-gd \
    -lvtkRenderingVolumeOpenGL2-9.2-gd \
    -lvtkRenderingVtkJS-9.2-gd \
    -lvtksqlite-9.2-gd \
    -lvtksys-9.2-gd \
    -lvtkTestingRendering-9.2-gd \
    -lvtktheora-9.2-gd \
    -lvtktiff-9.2-gd \
    -lvtkverdict-9.2-gd \
    -lvtkViewsContext2D-9.2-gd \
    -lvtkViewsCore-9.2-gd \
    -lvtkViewsInfovis-9.2-gd \
    -lvtkWrappingTools-9.2-gd \
    -lvtkzlib-9.2-gd \
    -lvtkGUISupportQt-9.2 \
    -lvtkGUISupportQtQuick-9.2 \
    -lvtkGUISupportQtSQL-9.2 \
    -lvtkRenderingQt-9.2 \
    -lvtkViewsQt-9.2

# opengl glfw
INCLUDEPATH += C:/Tools/OpenGL/glfw/include
DEPENDPATH += C:/Tools/OpenGL/glfw/include

LIBS += -lOpenGL32
LIBS += -LC:/Tools/OpenGL/glfw/lib-vc2022 -lglfw3dll
LIBS += -lglfw3dll\
        -lglfw3\
        -lglfw3_mt
# opengl glew
INCLUDEPATH += C:/Tools/OpenGL/glew/include
DEPENDPATH += C:/Tools/OpenGL/glew/include
LIBS += -LC:/Tools/OpenGL/glew/lib/Release/Win32
LIBS += -lglew32\
        -lglew32s
# opengl glut
INCLUDEPATH += C:/Tools/OpenGL/glut
DEPENDPATH += C:/Tools/OpenGL/glut
LIBS += -LC:/Tools/OpenGL/glut
LIBS += -lglut\
        -lglut32
QT += openglwidgets
