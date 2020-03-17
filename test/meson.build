test_names = [
	'cli',
	'linalg',
	'model',
	'obj2stlbin',
	'obj_parser',
]

test_incdirs = include_directories(
	'.',
)

foreach test_name : test_names
	test(
		test_name,
		executable(
			'test-' + test_name,
			[all_src_no_main, 'test_main.cpp', test_name + '_test.cpp'],
			include_directories : [incdirs, test_incdirs]
		),
		env: [
			'TEST_FILES_DIR=@0@'.format(join_paths(meson.source_root(),
				'test/test_files')),
		],
	)
endforeach
