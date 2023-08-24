project('ffms2', 'c', 'cpp',
  default_options : ['buildtype=release', 'warning_level=2', 'b_ndebug=if-release', 'c_std=c99', 'cpp_std=c++11'],
  meson_version : '>=0.60.3',
  version : run_command('version.sh', check: false).stdout().strip()
)

add_project_arguments('-DFFMS_EXPORTS', '-D_FILE_OFFSET_BITS=64', '-D__STDC_CONSTANT_MACROS', '-fno-omit-frame-pointer', language : ['c', 'cpp'])

deps = []

ffms2_sources = [
  'src/core/audiosource.cpp',
  'src/core/audiosource.h',
  'src/core/ffms.cpp',
  'src/core/filehandle.cpp',
  'src/core/filehandle.h',
  'src/core/indexing.cpp',
  'src/core/indexing.h',
  'src/core/track.cpp',
  'src/core/track.h',
  'src/core/utils.cpp',
  'src/core/utils.h',
  'src/core/videosource.cpp',
  'src/core/videosource.h',
  'src/core/videoutils.cpp',
  'src/core/videoutils.h',
  'src/core/zipfile.cpp',
  'src/core/zipfile.h'
]

if get_option('default_library') == 'shared' or get_option('default_library') == 'both'
  if get_option('vapoursynth').enabled()
    ffms2_vs = dependency('vapoursynth')
    if ffms2_vs.found()
      ffms2_sources += [
        'src/vapoursynth/vapoursource4.cpp',
        'src/vapoursynth/vapoursource4.h',
        'src/vapoursynth/vapoursynth4.cpp',
        'src/vapoursynth/VapourSynth4.h',
        'src/vapoursynth/VSHelper4.h',
        'src/vapoursynth/vapoursource.cpp',
        'src/vapoursynth/vapoursource.h',
        'src/vapoursynth/vapoursynth.cpp',
        'src/vapoursynth/VapourSynth.h',
        'src/vapoursynth/VSHelper.h'
      ]
      deps += ffms2_vs
    endif
  endif

  if get_option('avisynth-cpp').enabled()
    ffms2_avscpp = dependency('avisynth')
    if ffms2_avscpp.found()
      ffms2_sources += [
        'src/avisynth/avisynth.cpp',
        'src/avisynth/avssources.cpp',
        'src/avisynth/avssources.h'
      ]
      deps += ffms2_avscpp
    endif
  endif

  if get_option('avisynth-c').enabled()
    ffms2_sources += [
      'src/avisynth_c/avs/capi.h',
      'src/avisynth_c/avs/config.h',
      'src/avisynth_c/avs/types.h',
      'src/avisynth_c/avisynth.c',
      'src/avisynth_c/avisynth_c.h',
      'src/avisynth_c/avs_common.h',
      'src/avisynth_c/avs_convert.h',
      'src/avisynth_c/avs_utils.c',
      'src/avisynth_c/avs_utils.h',
      'src/avisynth_c/ff_audsource.c',
      'src/avisynth_c/ff_filters.h',
      'src/avisynth_c/ff_vidsource.c'
    ]
  endif

  if get_option('avisynth-cpp').enabled() and get_option('avisynth-c').enabled()
    error('Cannot build C and C++ AviSynth plugins in a single library!')
  endif
else
  if get_option('vapoursynth').enabled()
    error('VapourSynth plugin requires building FFMS2 as a shared library.')
  endif

  if get_option('avisynth-cpp').enabled()
    error('AviSynth plugin requires building FFMS2 as a shared library.')
  endif

  if get_option('avisynth-c').enabled()
    error('AviSynth plugin requires building FFMS2 as a shared library.')
  endif
endif

ffmsindex_sources = [
  'src/index/ffmsindex.cpp',
  'src/index/vsutf16.h'
]

includes = include_directories('include')

deps += [
  dependency('libavformat', version : '>=53.20.0'),
  dependency('libavcodec', version : '>=53.24.0'),
  dependency('libswresample', version : '>=1.0.0'),
  dependency('libswscale', version : '>=0.7.0'),
  dependency('libavutil', version : '>=51.21.0'),
  dependency('zlib')
]

if host_machine.cpu_family().endswith('x86')
  add_project_arguments('-m32', '-mfpmath=sse', '-msse', '-march=pentium3', '-mtune=pentium3', language : ['c', 'cpp'])
endif

ffms2_lib = library('ffms2', ffms2_sources,
  dependencies : deps,
  include_directories : includes,
  install : true,
  gnu_symbol_visibility : 'default',
  version: run_command('version.sh', check: false).stdout().strip()
)
ffms2_lib_dep = dependency('ffms2')

compiler = meson.get_compiler('cpp')
ffmsindex_ldflags = []
if (compiler.get_id() == 'gcc' or compiler.get_id() == 'clang') and host_machine.system() == 'windows'
  ffmsindex_ldflags += '-municode'
endif

executable('ffmsindex', ffmsindex_sources,
  link_args : ffmsindex_ldflags,
  link_with : ffms2_lib,
  dependencies : [deps, ffms2_lib_dep],
  include_directories : includes,
  install : true,
  gnu_symbol_visibility : 'default'
)

import('pkgconfig').generate(ffms2_lib,
  filebase : 'ffms2',
  name : 'ffms2',
  description : 'The Fabulous FM Library 2'
)

install_data(['doc/ffms2-api.md', 'doc/ffms2-changelog.md'],
  install_dir : join_paths(get_option('datadir'), 'doc', 'ffms2')
)

install_headers(['include/ffms.h', 'include/ffmscompat.h'])
