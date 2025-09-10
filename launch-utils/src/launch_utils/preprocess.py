import copy


MARKER_TAG = 'pragma.enable_preproc'
DEFAULT_TAG = 'pragma.default'
DERIVED_TAG = 'pragma.derived'

def does_eval_null(str):
    low = str.lower()
    return low == 'none' or low == 'null' or low == 'false'
def does_eval_default(str):
    low = str.lower()
    return low == 'default' or low == DEFAULT_TAG.lower()

def expand_dot_keys(d: dict) -> dict:
    """
    Expand keys containing '.' into nested dictionaries.
    Example: {"a.b.c": 1} -> {"a": {"b": {"c": 1}}}
    """
    result = {}
    for k, v in d.items():
        if "." in k:
            parts = k.split(".")
            current = result
            for part in parts[:-1]:
                current = current.setdefault(part, {})
            current[parts[-1]] = v
        else:
            result[k] = v
    return result


def deep_merge(base: dict, override: dict) -> dict:
    """
    Recursively merge override into base.
    - Dicts merge deeply
    - Any value of None in override shadows (removes) the key from base
    - Keys with '.' are expanded into nested dicts before merging
    """
    override = expand_dot_keys(override)
    result = copy.deepcopy(base)

    for k, v in override.items():
        if v is None:
            result.pop(k, None)
        elif isinstance(v, dict) and isinstance(result.get(k), dict):
            result[k] = deep_merge(result[k], v)
        else:
            result[k] = copy.deepcopy(v)

    return result


def prune_nulls(data):
    """
    Recursively remove keys with None values from dicts.
    """
    if isinstance(data, dict):
        return {k: prune_nulls(v) for k, v in data.items() if v is not None}
    elif isinstance(data, list):
        return [prune_nulls(v) for v in data]
    else:
        return data


def resolve_preset(block: dict, preset_name: str, seen=None) -> dict | None:
    """
    Resolve a preset in a block, handling inheritance (pragma.derived).
    Detects circular inheritance.
    """
    if preset_name is None:
        return None

    if preset_name not in block:
        raise ValueError(f"Preset '{preset_name}' not found in block {list(block.keys())}")

    seen = seen or set()
    if preset_name in seen:
        raise ValueError(f"Circular inheritance detected in preset '{preset_name}'")
    seen.add(preset_name)

    preset = copy.deepcopy(block[preset_name])

    if DERIVED_TAG in preset:
        parent_name = preset.pop(DERIVED_TAG)
        parent_config = resolve_preset(block, parent_name, seen)
        if parent_config is None:
            raise ValueError(f"Parent preset '{parent_name}' for '{preset_name}' is null")
        merged = deep_merge(parent_config, preset)
    else:
        merged = preset

    return prune_nulls(merged)


def preprocess_launch_json(config: dict, overrides: dict = None) -> dict:
    """
    Preprocess the whole config.
    overrides: optional dict {action_name: preset_name}
    """
    if MARKER_TAG not in config or not config[MARKER_TAG]:
        return config
    else:
        config.pop(MARKER_TAG)

    overrides = overrides or {}
    result = {}

    for action, block in config.items():
        default_preset = block.get(DEFAULT_TAG)
        chosen_preset = overrides.get(action, default_preset)

        if chosen_preset is None or does_eval_null(chosen_preset):
            chosen_present = None
        elif does_eval_default(chosen_preset):
            chosen_preset = default_preset

        resolved = resolve_preset(block, chosen_preset)
        if resolved is not None:
            result[action] = resolved
            print(f"[LAUNCH PREPROC]: Configured action '{action}' with preset '{chosen_preset}'")
        else:
            print(f"[LAUNCH PREPROC]: Removed action '{action}'")

    return result
