//
//  grabcutParameterForm.cpp
//  pcl
//
//  Created by Matthew Marzin on 11/18/14.
//  Copyright (c) 2014 Matthew Marzin. All rights reserved.
//

#include <pcl/apps/point_cloud_editor/grabcutParameterForm.h>

GrabcutParameterForm::GrabcutParameterForm () : ok_(false)
{
    k_line_ = new QLineEdit;
    lambda_line_ = new QLineEdit;
    button_box_ = new QDialogButtonBox;
    button_box_->addButton(tr("Cancel"),
                           QDialogButtonBox::RejectRole);
    button_box_->addButton(tr("OK"),
                           QDialogButtonBox::AcceptRole);
    connect(button_box_, SIGNAL(accepted()),
            this, SLOT(accept()));
    connect(button_box_, SIGNAL(rejected()),
            this, SLOT(reject()));
    layout_ = new QFormLayout;
    layout_->addRow(tr("&K:"), k_line_);
    layout_->addRow(tr("&Lambda:"),
                    lambda_line_);
    
    main_layout_ = new QVBoxLayout;
    main_layout_->addLayout(layout_);
    main_layout_->addWidget(button_box_);
    setLayout(main_layout_);
    setWindowTitle(tr("Grabcut Segmentation"));
}

GrabcutParameterForm::~GrabcutParameterForm ()
{
    delete k_line_;
    delete lambda_line_;
    delete button_box_;
    delete layout_;
    delete main_layout_;
}

void
GrabcutParameterForm::accept ()
{
    QString k_str = k_line_->text();
    bool ok;
    k_ = k_str.toFloat(&ok);
    // validates the input.
    if (!ok)
    {
        ok_ = false;
        return;
    }
    QString lambda_str = lambda_line_->text();
    lambda_ = lambda_str.toFloat(&ok);
    if (!ok)
    {
        ok_ = false;
        return;
    }
    this->done(0);
    ok_ = true;
}

void
GrabcutParameterForm::reject ()
{
    ok_ = false;
    this->done(0);
}

