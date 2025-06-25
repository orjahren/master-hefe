# TODO: Implementer denne
# TODO: Fastslå hvilket format vi bruker (OpenSCENARIO/CommonRoad/andre)
def file_format_is_valid(file_format: str) -> bool:
    """
    Check if the file format is valid.

    Args:
        file_format (str): The file format to check.

    Returns:
        bool: True if the file format is valid, False otherwise.
    """
    return file_format in ["json", "yaml", "yml", "csv", "txt"]
