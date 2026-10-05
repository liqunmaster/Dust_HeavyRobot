# Topic message definitions

`*.msg` files define plain C++ data structures shared between modules. Protocol
decoding and transport behavior remain in their existing modules.

Example:

```text
# @struct RemoteTopicData
float32 forward
uint8[8] payload
```

Supported types: `bool`, `int8`/`uint8` through `int64`/`uint64`, `float32`,
and `float64`. Fixed-size arrays use `type[count] name`.

Add each new `.msg` file and its output header to `Topic/CMakeLists.txt`. During
`west build`, `Script/tools/generate_messages.py` writes headers to
`build/generated/msg/`. Include them as `<msg/name.hpp>`; edit the `.msg` file,
not the generated header.
