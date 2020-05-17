#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>

#include <3dconv/cli.hpp>
#include <3dconv/model.hpp>
#include <3dconv/io.hpp>

int
main(int argc, char *argv[])
try {
	/* CLI parsing phase */
	CLIContext ctx{argc, argv};

	/* Model parsing phase */
	auto parser = IOMap<Parser>::get(ctx.itype());
	parser->open(ctx.ifile());
	auto model = (*parser)();
	model->validate();

	/* Print requested properties before transformation */
	print_properties(model, ctx.props());

	/* Transformation phase */
	if (!ctx.transforms().empty()) {
		std::cout << std::endl;
		std::cout << ">>> Performing transformations: " << ctx.transforms();

		auto tr_matrix = parse_transforms(ctx.transforms());
		model->transform(tr_matrix);

		std::cout << " Done." << std::endl;

		/* Print requested properties after transformation */
		if (ctx.props().any()) {
			std::cout << std::endl;
		}
		print_properties(model, ctx.props());
	}

	/* Model writing phase */
	auto writer = IOMap<Writer>::get(ctx.otype());
	writer->open(ctx.ofile());
	(*writer)(model);

	return 0;
} catch (const CLIError &ce) {
	std::cerr << "[ERROR | CLI] " << ce.what() << std::endl;
} catch (const ModelError &me) {
	std::cerr << "[ERROR | MODEL] " << me.what() << std::endl;
} catch (const ParseError &pe) {
	std::cerr << "[ERROR | PARSE | " << pe.filename << ":"
		<< pe.line_num << "] " << pe.what() << std::endl;
} catch (const WriteError &we) {
	std::cerr << "[ERROR | WRITE | " << we.filename << "] "
		<< we.what() << std::endl;
} catch (const std::exception &oe) {
	std::cerr << "[ERROR | OTHER] " << oe.what() << std::endl;
} catch (...) {
	std::cerr << "[ERROR | UNKNOWN]" << std::endl;
}
