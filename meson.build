project(
	'3dconv',
	'cpp',
	version : '0.1.0',
	license : 'BSD-3-Clause',
	default_options : ['cpp_std=c++17']
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
	'cli.cpp',
	'model.cpp',
]

all_src_no_main = []

foreach parser : parser_plugins
	all_src_no_main += join_paths(meson.source_root(),
		'plugins/parsers', parser)
endforeach

foreach writer : writer_plugins
	all_src_no_main += join_paths(meson.source_root(),
		'plugins/writers', writer)
endforeach

foreach source : sources_no_main
	all_src_no_main += join_paths(meson.source_root(), 'src', source)
endforeach

executable('3dconv', [all_src_no_main, 'src/main.cpp'],
	include_directories : incdirs)

subdir('doc')
subdir('test')
