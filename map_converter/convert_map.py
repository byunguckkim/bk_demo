import custom_map_converter

from simian.public.map import convert_map


def main():
    parser = convert_map.create_arg_parser()
    parser.add_argument(
        "--input_file",
        type = str,
        help = "path/to/input.file",
        required = False,
    )
    args = parser.parse_args()

    print("Starting map conversion. This will take multiple minutes...\n")
    convert_map.convert(custom_map_converter.convert, args)
    print("Map conversion complete!")


if __name__ == "__main__":
    main()
