test_names = [
	'bitset',
	'cli',
	'linalg',
	'model',
	'obj2stlbin',
	'obj_parser',
]

test_incdirs = include_directories(
	'.',
)

test_lib = library('test-lib', [src_no_main_paths, 'test_main.cpp'],
	include_directories : [incdirs, test_incdirs])

foreach test_name : test_names
	test(
		test_name,
		executable(
			'test-' + test_name,
			[plugin_paths, test_name + '_test.cpp'],
			include_directories : [incdirs, test_incdirs],
			link_with : test_lib
		),
		env: [
			'TEST_FILES_DIR=@0@'.format(join_paths(meson.source_root(),
				'test/test_files')),
		],
	)
endforeach
