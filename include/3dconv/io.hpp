#ifndef _3DCONV_IO_HPP
#define _3DCONV_IO_HPP

#include <filesystem>
#include <fstream>
#include <ios>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>

#include <3dconv/model.hpp>

/**
 * Base type for IO related errors.
 */
struct IOError : public std::logic_error {
	///
	IOError(const std::string &what_arg, const std::string &fn)
		: std::logic_error{what_arg}, filename{fn} {}
	std::string filename;
};

/**
 * On every parsing failure we can store the filename
 * and the line number of occurrence.
 */
struct ParseError : public IOError {
	///
	ParseError(const std::string &what_arg, const std::string &fn, size_t ln)
		: IOError(what_arg, fn), line_num{ln} {}
	size_t line_num;
};

/**
 * On every writing error we can store the filename.
 */
struct WriteError : public IOError {
	///
	WriteError(const std::string &what_arg, const std::string &fn)
		: IOError(what_arg, fn) {}
};

/**
 * Base class of Parser and Writer classes.
 */
class IO {
public:
	/** Creates a new std::fstream object and sets
	 * the file opening mode. */
	IO(std::ios_base::openmode omode) : openmode_{omode} {
		file_ = new std::fstream;
	}
	virtual ~IO() { delete file_; }

	// TODO: Error handling (exceptions)
	/** Opens the given file and stores the associated
	 * std::fstream in the object. */
	void open(std::string filename) {
		if (file_->is_open()) {
			file_->close();
		}
		filename_ = filename;
		file_->open(filename_, openmode_);
	}

	///
	void open(std::filesystem::path filepath) {
		this->open(filepath.string());
	}

protected:
	std::ios_base::openmode openmode_;
	std::string filename_;
	std::fstream *file_;
};

/**
 * Abstract IO handler class for parsers.
 */
class Parser : public IO {
public:
	/** Sets the default open mode for reading and
	 * accepts additional open modes. (e.g. binary)*/
	Parser(std::ios_base::openmode omode = std::ios_base::in)
		: IO{omode | std::ios_base::in} {}

	/** Call operator which should be implemented
	 * by the actual Parser specializations.
	 * Returns an std::shared_ptr to a Model object
	 * representing the result of the parsing. */
	virtual std::shared_ptr<Model> operator()() = 0;
};

/**
 * Abstract IO handler class for writers.
 */
class Writer : public IO {
public:
	/** Sets the default open mode for writing and
	 * accepts additional open modes. (e.g. binary)*/
	Writer(std::ios_base::openmode omode = std::ios_base::out)
		: IO{omode | std::ios_base::out | std::ios_base::trunc} {}

	/** Call operator which should be implemented
	 * by the actual Parser specializations.
	 * Accepts an std::shared_ptr to a previously
	 * constructed Model object. */
	virtual void operator()(const std::shared_ptr<const Model>) = 0;
};

/**
 * Class template for global Parser and Writer container singletons.
 *
 * Actually this is a map that contains a Parser/Writer instance for
 * every supported file types.
 *
 * <b>For plugin authors:</b> After implementing a Parser/Writer plugin
 * you should call the appropriate IOMap<...>::add() function before
 * any other part of the code wants to access the IOMaps. This can
 * easily be done via the REGISTER_PARSER() and REGISTER_WRITER() macros.
 */
template<typename BaseT,
	typename = std::enable_if_t<std::is_same_v<BaseT, Parser>
		|| std::is_same_v<BaseT, Writer>>>
class IOMap {
public:
	///
	template<typename SpecT,
		typename = std::enable_if_t<std::is_base_of_v<BaseT, SpecT>>>
	static int add(std::string file_type) {
		if (!instance_) {
			init();
		}

		std::shared_ptr<BaseT> ptr{new SpecT};
		instance_->map_.emplace(file_type, ptr);
		return 1;
	}

	///
	static std::shared_ptr<BaseT> get(std::string file_type) {
		if (!instance_) {
			init();
		}

		return instance_->map_.at(file_type);
	}

	///
	static const IOMap &instance() {
		if (!instance_) {
			init();
		}
		return *instance_;
	}

	///
	const auto map() const { return map_; }

private:
	inline static IOMap *instance_{nullptr};
	std::unordered_map<std::string, std::shared_ptr<BaseT>> map_;

	IOMap() {}
	static void init() { instance_ = new IOMap; }
};

/**
 * Helper macro for registering a new file type and
 * the corresponding Parser specialization into the
 * global IOMap<Parser>.
 */
#define REGISTER_PARSER(FILE_TYPE, PARSER_TYPE)\
static const auto _REG_PAR_RET_##FILE_TYPE_##PARSER_TYPE =\
	IOMap<Parser>::add<PARSER_TYPE>(FILE_TYPE)

/**
 * Helper macro for registering a new file type and
 * the corresponding Writer specialization into the
 * global IOMap<Writer>.
 */
#define REGISTER_WRITER(FILE_TYPE, WRITER_TYPE)\
static const auto _REG_WRI_RET_##FILE_TYPE_##WRITER_TYPE =\
	IOMap<Writer>::add<WRITER_TYPE>(FILE_TYPE)

#endif
