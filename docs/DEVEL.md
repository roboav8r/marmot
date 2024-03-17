# Adding a new detector

# Adding a new process model
To add a new process model:
- In `datatypes.py`: Add an appropriate entry eithin the `Track.__init__`, `Track.predict`, and `Track.compute_proc_model` definitions. 
- In `output.py`: Add 
- In `tracker.py`: Update `supported_proc_models`; add a `['obs_model']['proc_model']` entry for the process model; ensure `declare_obj_params` and `set_obj_properties` have entries for the necessary parameters;

# Other contributions and featuresIf 
For adding other features (new similarity metrics, two-stage association, non-max
