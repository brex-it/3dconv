# Plugins

We can easily write new plugins for input parsing and output writing. The only
thing we need to do is to create a new class which is derived from the Parser
or Writer classes depending on what kind of plugin we want to implement. It is
important that the implementation files of the plugins should be placed into
the `plugins/(parsers|writers)` subdirectories.

## Parsers

Every parser should override the call operator which returns a Model object
built from the iput file.

## Writers

The writers also should implement their own call operator but they take a
previously built Model object and write it to the output file.

## Registering plugins

Each of the parsers and writers should be registered into their corresponding
global IOMap instances, so they can be used by the rest of the program. This
can be done easily by calling the appropriate macro at the end of the plugin's
source file.

The two available macros are:

```cpp
REGISTER_PARSER(FILE_TYPE, PARSER_TYPE)
REGISTER_WRITER(FILE_TYPE, WRITER_TYPE)
```

An example usage:

```cpp
REGISTER_PARSER("obj", OBJParser);
```
