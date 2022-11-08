project('pdap', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3',
                     'cpp_std=c++20'])

subdir('src')

boost_dep = dependency('boost', modules :['program_options',])

exe = executable('pdap', sources,
  install : true,
  dependencies : boost_dep)

test('basic', exe)
