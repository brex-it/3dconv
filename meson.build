project(
	'3dconv',
	'cpp',
	version : '0.2.0',
	license : 'BSD-3-Clause',
	default_options : [
		'b_lto=true',
		'b_pie=true',
		'cpp_std=c++17',
		'debug=false',
		'optimization=2',
	]
)

incdirs = include_directories(
	'include',
)

parser_plugins = [
	'obj.cpp',
]

writer_plugins = [
	'stl_bin.cpp',
]

sources_no_main = [
	'bitset.cpp',
	'cli.cpp',
	'model.cpp',
]

plugin_paths = []
foreach parser : parser_plugins
	plugin_paths += files(join_paths('plugins/parsers', parser))
endforeach

foreach writer : writer_plugins
	plugin_paths += files(join_paths('plugins/writers', writer))
endforeach

src_no_main_paths = []
foreach source : sources_no_main
	src_no_main_paths += files(join_paths('src', source))
endforeach

executable('3dconv', [plugin_paths, src_no_main_paths, 'src/main.cpp'],
	include_directories : incdirs)

# Subdirs

subdir('doc')

if get_option('build_tests')
	subdir('test')
endif
