solution 'davis_sepia'
    configurations {'release', 'debug'}
    location 'build'
    project 'davis_sepia'
        kind 'ConsoleApp'
        language 'C++'
        location 'build'
        files {'source/*.hpp', 'test/*.cpp'}
        links {'usb-1.0'}
        defines {'SEPIA_COMPILER_WORKING_DIRECTORY="' .. project().location .. '"'}
        configuration 'release'
            targetdir 'build/release'
            defines {'NDEBUG'}
            flags {'OptimizeSpeed'}
        configuration 'debug'
            targetdir 'build/debug'
            defines {'DEBUG'}
            flags {'Symbols'}
        configuration 'linux'
            links {'pthread'}
            buildoptions {'-std=c++11'}
            linkoptions {'-std=c++11'}
        configuration 'macosx'
            includedirs {'/usr/local/include'}
            libdirs {'/usr/local/lib'}
            buildoptions {'-std=c++11'}
            linkoptions {'-std=c++11'}
