# -----------------------------------------------------------------------------
# Fly-in Makefile
# -----------------------------------------------------------------------------

PYTHON := .venv/bin/python
UV := uv

.PHONY: all install run debug clean lint lint-strict

all: install run

install:
	@echo "Setting up virtual environment and installing dependencies with uv..."
	@$(UV) venv --python 3.12 --allow-existing .venv
	@$(UV) pip install -e .

run:
	@if [ ! -f $(PYTHON) ]; then \
		echo "Virtual environment not found. Running install first..."; \
		$(MAKE) install; \
	fi
	@$(PYTHON) main.py maps/easy/01_linear_path.txt

debug:
	@if [ ! -f $(PYTHON) ]; then \
		echo "Virtual environment not found. Running install first..."; \
		$(MAKE) install; \
	fi
	@$(PYTHON) -m pdb main.py maps/easy/01_linear_path.txt

clean:
	@echo "Cleaning temporary build files and caches..."
	@rm -rf __pycache__ .mypy_cache .pytest_cache *.egg-info
	@find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	@find . -type f -name "*.pyc" -delete 2>/dev/null || true
	@find . -type f -name "*.pyo" -delete 2>/dev/null || true

lint:
	@echo "Running flake8 and mypy..."
	@if [ -f .venv/bin/flake8 ]; then \
		.venv/bin/flake8 .; \
		.venv/bin/mypy . --warn-return-any --warn-unused-ignores --ignore-missing-imports --disallow-untyped-defs --check-untyped-defs; \
	else \
		flake8 .; \
		mypy . --warn-return-any --warn-unused-ignores --ignore-missing-imports --disallow-untyped-defs --check-untyped-defs; \
	fi

lint-strict:
	@echo "Running strict flake8 and mypy checks..."
	@if [ -f .venv/bin/flake8 ]; then \
		.venv/bin/flake8 .; \
		.venv/bin/mypy . --strict; \
	else \
		flake8 .; \
		mypy . --strict; \
	fi
