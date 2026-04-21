import json
import logging
from pathlib import Path
from typing import Any, Optional


JsonType = dict[str, Any] | list[Any]

logger = logging.getLogger(__name__)


def log_json_data(
    file_path: str | Path,
    data: JsonType,
    append: bool = False,
    indent: int = 2,
    ensure_ascii: bool = False
) -> None:
    """
    Writes JSON data in the form of a dict or list.

    If append=True and the file already contains a JSON list, the new data
    will be appended. Otherwise the file will be overwritten.

    :param file_path: Path to the JSON file
    :param data: Dictionary or list to be written
    :param append: Whether to append to existing JSON list
    :param indent: Pretty print indentation
    :param ensure_ascii: Controls unicode handling

    :raises TypeError: If data is not JSON serializable
    :raises ValueError: If append=True but existing file structure is incompatible
    :raises OSError: File write issues
    """
    path = Path(file_path)

    try:
        path.parent.mkdir(parents=True, exist_ok=True)

        if append and path.exists():
            existing_data = get_json_data(path, default=[])

            if isinstance(existing_data, list):
                if isinstance(data, list):
                    existing_data.extend(data)
                else:
                    existing_data.append(data)

                data_to_write = existing_data
            else:
                raise ValueError(
                    f"Cannot append to non-list JSON structure in {path}"
                )
        else:
            data_to_write = data

        with path.open("w", encoding="utf-8") as f:
            json.dump(
                data_to_write,
                f,
                indent=indent,
                ensure_ascii=ensure_ascii
            )

    except TypeError:
        logger.exception("Provided data is not JSON serializable")
        raise

    except OSError:
        logger.exception("File write failure: %s", path)
        raise

    except Exception:
        logger.exception("Unexpected error while writing JSON")
        raise


def get_json_data(
    file_path: str | Path,
    default: Optional[JsonType] = None
) -> JsonType:
    """
    Reads JSON data from a file.

    :param file_path: Path to the JSON file
    :param default: Value returned if file does not exist

    :returns: Parsed JSON data

    :raises FileNotFoundError: If file missing and no default provided.
    :raises ValueError: Invalid JSON format.
    :raises OSError: File read issues.
    """
    path = Path(file_path)

    try:
        if not path.exists():
            if default is not None:
                return default
            raise FileNotFoundError(f"{path} does not exist")

        with path.open("r", encoding="utf-8") as f:
            return json.load(f)

    except json.JSONDecodeError as e:
        logger.exception("Invalid JSON format in %s", path)
        raise ValueError(f"Invalid JSON in {path}") from e

    except OSError:
        logger.exception("File read failure: %s", path)
        raise

    except Exception:
        logger.exception("Unexpected error while reading JSON")
        raise
