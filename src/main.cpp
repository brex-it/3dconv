#include <exception>
#include <iostream>

#include <3dconv/cli.hpp>
#include <3dconv/model.hpp>
#include <3dconv/io.hpp>

int
main(int argc, char *argv[])
try {
	/* CLI parsing phase */
	CLIContext ctx{argc, argv};
	InfoPrinter info{ctx.verbosity()};

	/* Model parsing phase */
	info(2, "Looking for I/O parser for file format: ", ctx.iformat());
	auto parser = IOMap<Parser>::get(ctx.iformat());
	info(2, "Opening file: ", ctx.ifile());
	parser->open(ctx.ifile());
	info(2, "Parsing and validating model from file: ", ctx.ifile());
	auto model = (*parser)();
	model->validate();

	/* Action runner loop */
	using AT = Action::ActionType;
	for (const auto &a : ctx.actions()) {
		switch (a.type) {
		case AT::PrintProperties:
			if (ctx.verbosity() > 0) {
				info(1, "Printing the requested properties: ", a.value);
				std::cout << std::endl;
				print_properties(model, a.value);
				std::cout << std::endl;
			}
			break;
		case AT::FaceTransform:
			{
				auto ft = parse_face_transforms(a.value);
				if (ft.convexify) {
					info(1, "Performing face convexification");
					model->convexify_faces();
				}
				if (ft.triangulate) {
					info(1, "Performing face triangulation");
					model->triangulate();
				}
			}
			break;
		case AT::ModelTransform:
			info(1, "Performing model transformations: ", a.value);
			auto tr_matrix = parse_model_transforms(a.value);
			model->transform(tr_matrix);
		}
	}

	/* Model writing phase */
	if (!ctx.ofile().empty()) {
		info(2, "Looking for I/O writer for file format: ", ctx.oformat());
		auto writer = IOMap<Writer>::get(ctx.oformat());
		info(2, "Opening file: ", ctx.ofile());
		writer->open(ctx.ofile());
		info(2, "Writing model to the file: ", ctx.ofile());
		(*writer)(model);
	}

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
} catch (const IOError &ie) {
	std::cerr << "[ERROR | I/O] " << ie.what() << std::endl;
} catch (const std::exception &oe) {
	std::cerr << "[ERROR | OTHER] " << oe.what() << std::endl;
} catch (...) {
	std::cerr << "[ERROR | UNKNOWN]" << std::endl;
}
