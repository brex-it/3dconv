doxygen = find_program('doxygen', required : false)

if doxygen.found()
	conf_data = configuration_data()
	src_root = meson.source_root()
	conf_data.set('DOXYGEN_INPUT',
		join_paths(src_root, 'include/3dconv') + ' '
		+ join_paths(src_root, 'plugins') + ' '
		+ join_paths(src_root, 'README.md')
	)
	conf_data.set('DOXYGEN_OUTPUT', meson.current_build_dir())
	conf_data.set('DOXYGEN_README', join_paths(src_root, 'README.md'))

	doxyfile = configure_file(
		input : 'Doxyfile.in',
		output : 'Doxyfile',
		configuration : conf_data
	)

	custom_target('doc',
		build_by_default : true,
		command : [doxygen, doxyfile],
		output : ['html']
	)
endif
