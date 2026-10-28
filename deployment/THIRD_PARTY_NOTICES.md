# Third-party components

This repository preserves the user's deployed application and its matching runtime artifacts. No blanket license grant for third-party models, vendor runtimes or firmware is implied by publication.

- RKNN runtime and RKNN-format models: Rockchip runtime plus the project's deployed trained/converted model artifacts. Original component terms continue to apply.
- RGA runtime: matching Rockchip RGA shared library from the deployed board.
- HyperLPR runtime: retained because it is a DT_NEEDED dependency of the frozen executable; the default mode does not initialize the OCR workflow.
- WenQuanYi Zen Hei font: the installed font notice is preserved under `licenses/`.
- OpenCV and Qt: supplied by the target OS packages. Available installed notices are preserved under `licenses/`; package identities are listed in `system-packages.tsv`.
- PCIe driver: the verified reference module and matching driver sources are kept under `driver/`; existing source headers and notices are preserved.

Credentials, SSH keys, browser state, machine authorization files and user capture datasets are not part of this repository.
