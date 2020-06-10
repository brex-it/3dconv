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
	auto parser = IOMap<Parser>::get(ctx.iformat());
	parser->open(ctx.ifile());
	auto model = (*parser)();
	model->validate();

	/* Action runner loop */
	using AT = Action::ActionType;
	for (const auto &a : ctx.actions()) {
		switch (a.type) {
		case AT::PrintProperties:
			std::cout << ">>> Printing the requested properties: " << a.value
				<< std::endl;
			std::cout << std::endl;
			print_properties(model, a.value);
			std::cout << std::endl;
			break;
		case AT::FaceTransform:
			{
				auto ft = parse_face_transforms(a.value);
				if (ft.convexify) {
					std::cout << ">>> Performing face convexification"
						<< std::endl;
					model->convexify_faces();
				}
				if (ft.triangulate) {
					std::cout << ">>> Performing face triangulation"
						<< std::endl;
					model->triangulate();
				}
			}
			break;
		case AT::ModelTransform:
			std::cout << ">>> Performing model transformations: " << a.value
				<< std::endl;
			auto tr_matrix = parse_model_transforms(a.value);
			model->transform(tr_matrix);
		}
	}

	/* Model writing phase */
	auto writer = IOMap<Writer>::get(ctx.oformat());
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
